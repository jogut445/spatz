// Copyright 2023 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Matheus Cavalcante, ETH Zurich
//
// The register file stores all vectors, organized into banks.

module spatz_vrf
  import spatz_pkg::*;
  import schnizo_pkg::*;
  #(
    parameter int unsigned NrReadPorts  = 5,
    parameter int unsigned NrWritePorts = 3,
    parameter int unsigned FpuBufDepth  = 4,
    // WAR hazard tracking parameters
    parameter int unsigned NofVLSU    = 0,
    parameter int unsigned NofVFU     = 0,
    parameter int unsigned VlsuNofRss = 0,
    parameter int unsigned VfuNofRss  = 0,
    parameter bit SIMD = 1'b0
  ) (
    input  logic                         clk_i,
    input  logic                         rst_ni,
    input  logic                         testmode_i,
    // Write ports
    input  vrf_addr_t [NrWritePorts-1:0] waddr_i,
    input  vrf_data_t [NrWritePorts-1:0] wdata_i,
    input  logic      [NrWritePorts-1:0] we_i,
    input  vrf_be_t   [NrWritePorts-1:0] wbe_i,
    output logic      [NrWritePorts-1:0] wvalid_o,
    // Loop state for WAR hazard tracking
    input  loop_state_e                  loop_state_i,
`ifdef BUF_FPU
    // Signal to track if  result can be buffered or not
    input  logic      [$clog2(FpuBufDepth)-1:0] fpu_buf_usage_i,
`endif
    // Read ports
    input  vrf_addr_t [NrReadPorts-1:0]  raddr_i,
    input  logic      [NrReadPorts-1:0]  re_i,
    // First beat of a new read transaction per port (see WAR tracking comment below)
    input  logic      [NrReadPorts-1:0]  re_first_i,
    output vrf_data_t [NrReadPorts-1:0]  rdata_o,
    output logic      [NrReadPorts-1:0]  rvalid_o
  );

`include "common_cells/registers.svh"

  ////////////////
  // Parameters //
  ////////////////

  localparam int unsigned NrReadPortsPerBank = 3;

  // WAR hazard tracking
  localparam int unsigned MaxConsumers = VlsuNofRss * NofVLSU + VfuNofRss * NofVFU;
  localparam int unsigned CntW         = $clog2(MaxConsumers + 1);
  localparam int unsigned ReadCntW     = $clog2(NrReadPorts + 1);

  //////////////
  // Typedefs //
  //////////////

  typedef logic [$bits(vrf_addr_t)-$clog2(NrVRFBanks)-1:0] vregfile_addr_t;

  function automatic logic [$clog2(NrWordsPerBank)-1:0] f_vreg(vrf_addr_t addr);
    f_vreg = addr[$clog2(NrVRFWords)-1:$clog2(NrVRFBanks)];
  endfunction: f_vreg

  function automatic logic [$clog2(NrVRFBanks)-1:0] f_bank(vrf_addr_t addr);
`ifdef DOUBLE_BW
    // Use a simple vector register to bank mapping
    // No particular performance benefit from barber pole layout since 3R ports per bank is already available
    f_bank = addr[$clog2(NrVRFBanks)-1:0];
`else
    // Is this vreg divisible by eight?
    automatic logic [1:0] vreg8 = addr[$clog2(8*NrWordsPerVector) +: 2];

    // Barber's pole. Advance the starting bank of each vector by one every eight vector registers.
    f_bank = addr[$clog2(NrVRFBanks)-1:0] + vreg8;
`endif
  endfunction: f_bank

  /////////////
  // Signals //
  /////////////

  // Write signals
  vregfile_addr_t [NrVRFBanks-1:0] waddr;
  vrf_data_t      [NrVRFBanks-1:0] wdata;
  logic           [NrVRFBanks-1:0] we;
  vrf_be_t        [NrVRFBanks-1:0] wbe;

  // Signals to handle conflicts between VFU and VLSU interfaces
  logic           [NrVRFBanks-1:0] w_vlsu_vfu_conflict;
  logic           [NrVRFBanks-1:0] w_vfu;

  // Read signals
  vregfile_addr_t [NrVRFBanks-1:0][NrReadPortsPerBank-1:0] raddr;
  vrf_data_t      [NrVRFBanks-1:0][NrReadPortsPerBank-1:0] rdata;

  // WAR hazard tracking state (per VRF register address).
  // Declared in outer scope because gen_lep_write_blocked references them;
  // driven by FFs inside gen_simd_war_tracking or tied to 0 in the else branch.
  logic                          [NrVRFWords-1:0] tracking_active_q;
  logic [NrVRFWords-1:0][CntW-1:0]               consumer_count_q;
  logic [NrVRFWords-1:0][CntW-1:0]               consumed_by_q;
  logic [NrVRFWords-1:0][CntW-1:0]               consumed_by_d;
  logic                          [NrVRFWords-1:0] consumed_by_frozen_q;
  // Write-blocked mask for LEP gating (combinational)
  logic [NrWritePorts-1:0]                        lep_write_blocked;
  // Old-value fully-consumed mask (combinational). Set for a tracked register
  // when every consumer of the value it currently holds has completed *as of
  // this cycle* (registered consumed_by minus this cycle's last-beat reads == 0).
  // Drives BOTH the write unblock and the read stall, so the producer write is
  // released in the very cycle the reference count reaches zero (no bubble) and
  // reads are held until that write commits (no stale RAW read). Driven
  // write-independently inside gen_simd_war_tracking -> no comb loop with wvalid_o.
  logic                          [NrVRFWords-1:0] lep_old_drained;

  ///////////////////
  // Write Mapping //
  ///////////////////

  // Block writes to tracked registers that still have pending consumers of the
  // value they currently hold. Released the cycle the reference count reaches
  // zero (lep_old_drained, which already accounts for this cycle's reads) so the
  // write commits in that same cycle with no bubble. Only active in SIMD mode;
  // in standard Spatz always 0.
  //
  // Covers BOTH LoopLcp2 and LoopLep, symmetric with the combinatorial read-stall
  // below (which already spans {LoopLcp2, LoopLep}). The previous LoopLep-only
  // gate left a hole in LCP2: with the pipelined VLSU load->load hand-off, a
  // producer write can become ready a cycle early and commit before the old
  // value's consumers have read it - corrupting the value (seen as a whole-row
  // error in vfu_test_gemm_3x). The consumed_by_frozen_q guard restricts the
  // block to registers whose consumer count is meaningful: an unfrozen reg in
  // LCP2 still has consumed_by_q == 0 (so lep_old_drained would be spuriously
  // true anyway) and must not block its own freezing first write. In LEP every
  // tracked reg is already frozen, so the guard is a no-op there.
  always_comb begin : gen_lep_write_blocked
    lep_write_blocked = '0;
    if (SIMD && (loop_state_i inside {LoopLcp2, LoopLep})) begin
      for (int p = 0; p < NrWritePorts; p++) begin
        if (we_i[p] && tracking_active_q[waddr_i[p]]
                    && consumed_by_frozen_q[waddr_i[p]]
                    && !lep_old_drained[waddr_i[p]])
          lep_write_blocked[p] = 1'b1;
      end
    end
  end : gen_lep_write_blocked

  logic [NrVRFBanks-1:0][NrWritePorts-1:0] write_request;
  always_comb begin: gen_write_request
    for (int bank = 0; bank < NrVRFBanks; bank++) begin
      for (int port = 0; port < NrWritePorts; port++) begin
        write_request[bank][port] = we_i[port] && !lep_write_blocked[port]
                                    && f_bank(waddr_i[port]) == bank;
      end
    end
  end: gen_write_request

  always_comb begin : proc_write
    waddr    = '0;
    wdata    = '0;
    we       = '0;
    wbe      = '0;
    wvalid_o = '0;

    // For each bank, we have a priority based access scheme. First priority always has the VFU,
    // second priority has the LSU, and third priority has the slide unit.
    for (int unsigned bank = 0; bank < NrVRFBanks; bank++) begin
`ifdef BUF_FPU
`ifdef DOUBLE_BW
      automatic logic write_request_vlsu = write_request[bank][VLSU_VD_WD0] | write_request[bank][VLSU_VD_WD1];
`else
      automatic logic write_request_vlsu = write_request[bank][VLSU_VD_WD];
`endif
      w_vlsu_vfu_conflict[bank] = write_request_vlsu & write_request[bank][VFU_VD_WD];
      // Prioritize VFU when VFU buffer usage is high
      // Otherwise VLSU gets the priority
      w_vfu[bank] = w_vlsu_vfu_conflict[bank] && (fpu_buf_usage_i >= (FpuBufDepth-2));
`else
      // If no buffering is done, prioritize VFU always
      w_vfu[bank] = 1'b1;
`endif

`ifdef DOUBLE_BW
      if (~w_vfu[bank]) begin
        // Prioritize VLSU interfaces
        if (write_request[bank][VLSU_VD_WD0]) begin
          waddr[bank]          = f_vreg(waddr_i[VLSU_VD_WD0]);
          wdata[bank]          = wdata_i[VLSU_VD_WD0];
          we[bank]             = 1'b1;
          wbe[bank]            = wbe_i[VLSU_VD_WD0];
          wvalid_o[VLSU_VD_WD0] = 1'b1;
        end else if (write_request[bank][VLSU_VD_WD1]) begin
          waddr[bank]          = f_vreg(waddr_i[VLSU_VD_WD1]);
          wdata[bank]          = wdata_i[VLSU_VD_WD1];
          we[bank]             = 1'b1;
          wbe[bank]            = wbe_i[VLSU_VD_WD1];
          wvalid_o[VLSU_VD_WD1] = 1'b1;
        end else if (write_request[bank][VFU_VD_WD]) begin
          waddr[bank]         = f_vreg(waddr_i[VFU_VD_WD]);
          wdata[bank]         = wdata_i[VFU_VD_WD];
          we[bank]            = 1'b1;
          wbe[bank]           = wbe_i[VFU_VD_WD];
          wvalid_o[VFU_VD_WD] = 1'b1;
        end else if (write_request[bank][VSLDU_VD_WD]) begin
          waddr[bank]           = f_vreg(waddr_i[VSLDU_VD_WD]);
          wdata[bank]           = wdata_i[VSLDU_VD_WD];
          we[bank]              = 1'b1;
          wbe[bank]             = wbe_i[VSLDU_VD_WD];
          wvalid_o[VSLDU_VD_WD] = 1'b1;
        end
      end else begin
        // Prioritize VFU
        if (write_request[bank][VFU_VD_WD]) begin
          waddr[bank]         = f_vreg(waddr_i[VFU_VD_WD]);
          wdata[bank]         = wdata_i[VFU_VD_WD];
          we[bank]            = 1'b1;
          wbe[bank]           = wbe_i[VFU_VD_WD];
          wvalid_o[VFU_VD_WD] = 1'b1;
        end else if (write_request[bank][VLSU_VD_WD0]) begin
          waddr[bank]          = f_vreg(waddr_i[VLSU_VD_WD0]);
          wdata[bank]          = wdata_i[VLSU_VD_WD0];
          we[bank]             = 1'b1;
          wbe[bank]            = wbe_i[VLSU_VD_WD0];
          wvalid_o[VLSU_VD_WD0] = 1'b1;
        end else if (write_request[bank][VLSU_VD_WD1]) begin
          waddr[bank]          = f_vreg(waddr_i[VLSU_VD_WD1]);
          wdata[bank]          = wdata_i[VLSU_VD_WD1];
          we[bank]             = 1'b1;
          wbe[bank]            = wbe_i[VLSU_VD_WD1];
          wvalid_o[VLSU_VD_WD1] = 1'b1;
        end else if (write_request[bank][VSLDU_VD_WD]) begin
          waddr[bank]           = f_vreg(waddr_i[VSLDU_VD_WD]);
          wdata[bank]           = wdata_i[VSLDU_VD_WD];
          we[bank]              = 1'b1;
          wbe[bank]             = wbe_i[VSLDU_VD_WD];
          wvalid_o[VSLDU_VD_WD] = 1'b1;
        end
      end
`else
      if (~w_vfu[bank]) begin
        // Prioritize VLSU interfaces
        if (write_request[bank][VLSU_VD_WD]) begin
          waddr[bank]          = f_vreg(waddr_i[VLSU_VD_WD]);
          wdata[bank]          = wdata_i[VLSU_VD_WD];
          we[bank]             = 1'b1;
          wbe[bank]            = wbe_i[VLSU_VD_WD];
          wvalid_o[VLSU_VD_WD] = 1'b1;
        end else if (write_request[bank][VFU_VD_WD]) begin
          waddr[bank]         = f_vreg(waddr_i[VFU_VD_WD]);
          wdata[bank]         = wdata_i[VFU_VD_WD];
          we[bank]            = 1'b1;
          wbe[bank]           = wbe_i[VFU_VD_WD];
          wvalid_o[VFU_VD_WD] = 1'b1;
        end else if (write_request[bank][VSLDU_VD_WD]) begin
          waddr[bank]           = f_vreg(waddr_i[VSLDU_VD_WD]);
          wdata[bank]           = wdata_i[VSLDU_VD_WD];
          we[bank]              = 1'b1;
          wbe[bank]             = wbe_i[VSLDU_VD_WD];
          wvalid_o[VSLDU_VD_WD] = 1'b1;
        end
      end else begin
        // Prioritize VFU
        if (write_request[bank][VFU_VD_WD]) begin
          waddr[bank]         = f_vreg(waddr_i[VFU_VD_WD]);
          wdata[bank]         = wdata_i[VFU_VD_WD];
          we[bank]            = 1'b1;
          wbe[bank]           = wbe_i[VFU_VD_WD];
          wvalid_o[VFU_VD_WD] = 1'b1;
        end else if (write_request[bank][VLSU_VD_WD]) begin
          waddr[bank]          = f_vreg(waddr_i[VLSU_VD_WD]);
          wdata[bank]          = wdata_i[VLSU_VD_WD];
          we[bank]             = 1'b1;
          wbe[bank]            = wbe_i[VLSU_VD_WD];
          wvalid_o[VLSU_VD_WD] = 1'b1;
        end else if (write_request[bank][VSLDU_VD_WD]) begin
          waddr[bank]           = f_vreg(waddr_i[VSLDU_VD_WD]);
          wdata[bank]           = wdata_i[VSLDU_VD_WD];
          we[bank]              = 1'b1;
          wbe[bank]             = wbe_i[VSLDU_VD_WD];
          wvalid_o[VSLDU_VD_WD] = 1'b1;
        end
      end
`endif
    end
  end

  //////////////////
  // Read Mapping //
  //////////////////

  logic [NrVRFBanks-1:0][NrReadPorts-1:0] read_request;
  always_comb begin: gen_read_request
    for (int bank = 0; bank < NrVRFBanks; bank++) begin
      for (int port = 0; port < NrReadPorts; port++) begin
        // In LEP: stall reads to a tracked register while consumed_by == 0.
        // consumed_by == 0 means all consumers for the current iteration have
        // finished but the write has not yet fired to reset it; letting the next
        // iteration's read through now would leave consumed_by permanently at 0
        // and block the subsequent write forever.
        // automatic logic lep_read_stall = '0;
        automatic logic lep_read_stall = SIMD && (loop_state_i == LoopLep)
                                         && tracking_active_q[raddr_i[port]]
                                         && (consumed_by_q[raddr_i[port]] == '0);
        read_request[bank][port] = re_i[port] && !lep_read_stall
                                   && f_bank(raddr_i[port]) == bank;
      end
    end
  end: gen_read_request

  always_comb begin : proc_read
    raddr    = '0;
    rvalid_o = '0;
    rdata_o  = 'x;

    // For each port or each bank we have a priority based access scheme.
    // Port zero can only be accessed by the VFU (vs2). Port one can be accessed by
    // the VFU (vs1) and then by the slide unit. Port two can be accessed first by the
    // VFU (vd), then by the LSU.
    for (int unsigned bank = 0; bank < NrVRFBanks; bank++) begin
      // Bank read port 0 - Priority: VFU (2) -> VLSU
      if (read_request[bank][VFU_VS2_RD]) begin
        raddr[bank][0]       = f_vreg(raddr_i[VFU_VS2_RD]);
        rdata_o[VFU_VS2_RD]  = rdata[bank][0];
        rvalid_o[VFU_VS2_RD] = 1'b1;
      end
`ifdef DOUBLE_BW
      else if (read_request[bank][VLSU_VD_RD0]) begin
        raddr[bank][0]        = f_vreg(raddr_i[VLSU_VD_RD0]);
        rdata_o[VLSU_VD_RD0]  = rdata[bank][0];
        rvalid_o[VLSU_VD_RD0] = 1'b1;
      end
`else
      else if (read_request[bank][VLSU_VS2_RD]) begin
        raddr[bank][0]        = f_vreg(raddr_i[VLSU_VS2_RD]);
        rdata_o[VLSU_VS2_RD]  = rdata[bank][0];
        rvalid_o[VLSU_VS2_RD] = 1'b1;
      end
`endif

      // Bank read port 1 - Priority: VFU (1) -> VLSU -> VSLDU
      if (read_request[bank][VFU_VS1_RD]) begin
        raddr[bank][1]       = f_vreg(raddr_i[VFU_VS1_RD]);
        rdata_o[VFU_VS1_RD]  = rdata[bank][1];
        rvalid_o[VFU_VS1_RD] = 1'b1;
      end
`ifdef DOUBLE_BW
      else if (read_request[bank][VLSU_VD_RD1]) begin
        raddr[bank][1]         = f_vreg(raddr_i[VLSU_VD_RD1]);
        rdata_o[VLSU_VD_RD1]  = rdata[bank][1];
        rvalid_o[VLSU_VD_RD1] = 1'b1;
      end
`endif
      else if (read_request[bank][VSLDU_VS2_RD]) begin
        raddr[bank][1]         = f_vreg(raddr_i[VSLDU_VS2_RD]);
        rdata_o[VSLDU_VS2_RD]  = rdata[bank][1];
        rvalid_o[VSLDU_VS2_RD] = 1'b1;
      end

      // Bank read port 2 - Priority: VFU (D) -> VLSU
      if (read_request[bank][VFU_VD_RD]) begin
        raddr[bank][2]      = f_vreg(raddr_i[VFU_VD_RD]);
        rdata_o[VFU_VD_RD]  = rdata[bank][2];
        rvalid_o[VFU_VD_RD] = 1'b1;
      end
`ifdef DOUBLE_BW
      // VLSU indices
      else if (read_request[bank][VLSU_VS2_RD0]) begin
        raddr[bank][2]        = f_vreg(raddr_i[VLSU_VS2_RD0]);
        rdata_o[VLSU_VS2_RD0]  = rdata[bank][2];
        rvalid_o[VLSU_VS2_RD0] = 1'b1;
      end else if (read_request[bank][VLSU_VS2_RD1]) begin
        raddr[bank][2]        = f_vreg(raddr_i[VLSU_VS2_RD1]);
        rdata_o[VLSU_VS2_RD1]  = rdata[bank][2];
        rvalid_o[VLSU_VS2_RD1] = 1'b1;
      end
`else
      else if (read_request[bank][VLSU_VD_RD]) begin
        raddr[bank][2]       = f_vreg(raddr_i[VLSU_VD_RD]);
        rdata_o[VLSU_VD_RD]  = rdata[bank][2];
        rvalid_o[VLSU_VD_RD] = 1'b1;
      end
`endif
    end

    // Combinatorial WAR/RAW handling for the loop epilogue.
    //
    // Once the value a tracked register currently holds is fully consumed
    // (lep_old_drained) its next value must come from the pending producer write
    // - any read of that register has to take that new value, never the stale SCM
    // contents. Per read of a drained register, in the SAME cycle:
    //   * if the producer write is committing -> byte-merge its data over the SCM
    //     read and assert rvalid_o (read served the very cycle the write fires);
    //   * otherwise -> hold rvalid_o low (stall) until the write commits.
    //
    // The stall is gated on re_i, NOT re_first_i: re_first_i is a single-cycle
    // pulse, so a read that has to wait several cycles for the producer would slip
    // through on every cycle after the first. re_i stays high for the whole read
    // transaction, so the decision is re-evaluated every cycle the read is
    // outstanding. A stalled read drives rvalid_o=0, so it is never counted as a
    // last-beat (rvalid_q stays 0) and cannot corrupt the consumer count; it is
    // charged to the new value only once it is actually served via forwarding.
    //
    // Covers LoopLcp2 as well as LoopLep: the consumed_by count is already frozen
    // and decrementing in LCP2, and a "first-LEP" read can be sampled in the
    // boundary cycle while loop_state_i still reads LoopLcp2 - gating on LoopLep
    // alone would let exactly that read escape the stall.
    if (SIMD && (loop_state_i inside {LoopLcp2, LoopLep})) begin
      for (int p = 0; p < NrReadPorts; p++) begin
        // consumed_by_frozen_q gates out tracked regs still in their LCP2
        // counting phase: there consumed_by_q is 0 (not yet frozen) so
        // lep_old_drained would be spuriously true and stall the very reads that
        // must be counted. Once frozen, the count is meaningful and the drain
        // stall applies (and in LEP every tracked reg is already frozen).
        if (re_i[p] && tracking_active_q[raddr_i[p]] && consumed_by_frozen_q[raddr_i[p]]
                    && lep_old_drained[raddr_i[p]]) begin
          automatic logic committing = 1'b0;
          for (int w = 0; w < NrWritePorts; w++) begin
            if (wvalid_o[w] && (waddr_i[w] == raddr_i[p])) begin
              // Byte-wise merge honouring the write byte-enable; unwritten bytes
              // keep the (correct, unaffected) SCM read data.
              for (int b = 0; b < $bits(vrf_be_t); b++) begin
                if (wbe_i[w][b])
                  rdata_o[p][b*8 +: 8] = wdata_i[w][b*8 +: 8];
              end
              committing = 1'b1;
            end
          end
          // Drained read: serve the forwarded write this cycle, else stall.
          rvalid_o[p] = committing;
        end
      end
    end
  end

  ///////////////////////
  // WAR Hazard Tracking
  ///////////////////////
  // Active only when SIMD=1 (Schnizo); in standard Spatz all tracking state is 0.

  if (SIMD) begin : gen_simd_war_tracking

    // Registered rvalid/raddr to detect the last beat of each read transaction.
    // last_read[p] fires the cycle AFTER the final beat: when re_first_i signals a new
    // dispatch (back-to-back reads, re never drops) or when re_i simply falls.
    logic      [NrReadPorts-1:0] rvalid_q;
    vrf_addr_t [NrReadPorts-1:0] raddr_q;
    logic      [NrReadPorts-1:0] last_read;
    // Per-address last-beat count (combinational, counted one cycle late)
    logic [NrVRFWords-1:0][ReadCntW-1:0] rvalid_per_addr;

    logic [NrVRFWords-1:0]           tracking_active_d;
    logic [NrVRFWords-1:0][CntW-1:0] consumer_count_d;
    logic [NrVRFWords-1:0]           consumed_by_frozen_d;

    // Detect last beat of each read transaction:
    //   - re_i drops (normal end of read)
    //   - re_first_i asserted while re was high (new dispatch arrived, back-to-back reads)
    always_comb begin : gen_last_read
      for (int p = 0; p < NrReadPorts; p++)
        last_read[p] = rvalid_q[p] & (~re_i[p] | re_first_i[p]);
    end : gen_last_read

    // Count one read per register address at the last beat only, using registered address.
    always_comb begin : gen_rvalid_per_addr
      rvalid_per_addr = '0;
      for (int p = 0; p < NrReadPorts; p++) begin
        if (last_read[p])
          rvalid_per_addr[raddr_q[p]] = rvalid_per_addr[raddr_q[p]] + ReadCntW'(1);
      end
    end : gen_rvalid_per_addr

    // Old value fully consumed as of this cycle: registered consumed_by minus
    // this cycle's last-beat reads == 0. Purely a function of registered state
    // and rvalid_per_addr (which derives from rvalid_q), so it does NOT depend on
    // wvalid_o -> safe to feed the write unblock without a combinational loop.
    always_comb begin : gen_lep_old_drained
      lep_old_drained = '0;
      for (int r = 0; r < NrVRFWords; r++) begin
        if (tracking_active_q[r]) begin
          automatic logic [CntW-1:0] cby_after =
            (consumed_by_q[r] >= CntW'(rvalid_per_addr[r]))
              ? consumed_by_q[r] - CntW'(rvalid_per_addr[r])
              : '0;
          lep_old_drained[r] = (cby_after == '0);
        end
      end
    end : gen_lep_old_drained

    always_comb begin : proc_war_tracking
      tracking_active_d    = tracking_active_q;
      consumer_count_d     = consumer_count_q;
      consumed_by_d        = consumed_by_q;
      consumed_by_frozen_d = consumed_by_frozen_q;

      // Reset all tracking state when returning to non-LxP execution
      if (loop_state_i inside {LoopRegular, LoopHwLoop}) begin
        tracking_active_d    = '0;
        consumer_count_d     = '0;
        consumed_by_d        = '0;
        consumed_by_frozen_d = '0;
      end else begin

        // ---- LCP1: start tracking on first write ----
        if (loop_state_i == LoopLcp1) begin
          for (int p = 0; p < NrWritePorts; p++) begin
            if (wvalid_o[p]) begin
              automatic int unsigned r = unsigned'(waddr_i[p]);
              if (!tracking_active_q[r]) begin
                tracking_active_d[r]    = 1'b1;
                consumer_count_d[r]     = '0;
                consumed_by_frozen_d[r] = 1'b0;
              end
            end
          end

          // Count reads to tracked registers (consumer count per loop body pass)
          for (int r = 0; r < NrVRFWords; r++) begin
            if (tracking_active_q[r] && !consumed_by_frozen_q[r] && rvalid_per_addr[r] != '0)
              consumer_count_d[r] = consumer_count_q[r] + CntW'(rvalid_per_addr[r]);
          end
        end

        // ---- LCP2: freeze consumer_count on first write, then count down consumed_by on reads ----
        if (loop_state_i == LoopLcp2) begin
          for (int p = 0; p < NrWritePorts; p++) begin
            if (wvalid_o[p]) begin
              automatic int unsigned r = unsigned'(waddr_i[p]);
              if (tracking_active_q[r] && !consumed_by_frozen_q[r]) begin
                consumed_by_d[r]        = consumer_count_q[r];
                consumed_by_frozen_d[r] = 1'b1;
              end
            end
          end

          for (int r = 0; r < NrVRFWords; r++) begin
            // A committing write to an ALREADY-frozen register (e.g. the next
            // producer write released by gen_lep_write_blocked in LCP2 once the
            // old value drained - reachable now that the write-block spans LCP2
            // for the pipelined VLSU hand-off). Re-arm exactly like LEP: the reads
            // this cycle are the last consumers of the OLD value and must not be
            // charged against the freshly written value. The freezing first write
            // (frozen_q==0) is handled by the write loop above and is excluded here.
            automatic logic has_frozen_write = 1'b0;
            for (int p = 0; p < NrWritePorts; p++) begin
              if (wvalid_o[p] && tracking_active_q[r] && consumed_by_frozen_q[r]
                              && (unsigned'(waddr_i[p]) == r))
                has_frozen_write = 1'b1;
            end

            if (has_frozen_write) begin
              consumed_by_d[r] = consumer_count_q[r];
            end else if (tracking_active_q[r] && rvalid_per_addr[r] != '0) begin
              if (!consumed_by_frozen_q[r])
                consumer_count_d[r] = consumer_count_q[r] + CntW'(rvalid_per_addr[r]);
              else begin
                if (consumed_by_q[r] >= CntW'(rvalid_per_addr[r]))
                  consumed_by_d[r] = consumed_by_q[r] - CntW'(rvalid_per_addr[r]);
                else
                  consumed_by_d[r] = '0;
              end
            end
          end
        end

        // ---- LEP: gate writes; decrement consumed_by on reads; reset on committed write ----
        if (loop_state_i == LoopLep) begin
          for (int r = 0; r < NrVRFWords; r++) begin
            automatic logic has_write = 1'b0;
            for (int p = 0; p < NrWritePorts; p++) begin
              if (wvalid_o[p] && tracking_active_q[r] && (unsigned'(waddr_i[p]) == r))
                has_write = 1'b1;
            end

            if (has_write) begin
              // The write commits the cycle the count reaches zero (lep_old_drained).
              // The reads counted this cycle (rvalid_per_addr) are the LAST consumers
              // of the OLD value -- they are what drove the count to zero -- so they
              // must NOT be charged against the freshly written value. Re-arm to the
              // full consumer_count; consumers of the new value (including a read
              // forwarded this same cycle) are counted on their own last beat.
              consumed_by_d[r] = consumer_count_q[r];
            end else if (tracking_active_q[r] && rvalid_per_addr[r] != '0) begin
              if (consumed_by_q[r] >= CntW'(rvalid_per_addr[r]))
                consumed_by_d[r] = consumed_by_q[r] - CntW'(rvalid_per_addr[r]);
              else
                consumed_by_d[r] = '0;
            end
          end
        end

      end // not LoopRegular/LoopHwLoop
    end : proc_war_tracking

    `FF(tracking_active_q,    tracking_active_d,    '0, clk_i, rst_ni)
    `FF(consumer_count_q,     consumer_count_d,     '0, clk_i, rst_ni)
    `FF(consumed_by_q,        consumed_by_d,        '0, clk_i, rst_ni)
    `FF(consumed_by_frozen_q, consumed_by_frozen_d, '0, clk_i, rst_ni)
    `FF(rvalid_q,             rvalid_o,             '0, clk_i, rst_ni)
    `FF(raddr_q,              raddr_i,              '0, clk_i, rst_ni)

  end else begin : gen_no_simd_war_tracking

    assign tracking_active_q    = '0;
    assign consumer_count_q     = '0;
    assign consumed_by_q        = '0;
    assign consumed_by_d        = '0;
    assign consumed_by_frozen_q = '0;
    assign lep_old_drained      = '0;

  end : gen_no_simd_war_tracking

  ////////////////
  // VREG Banks //
  ////////////////

  for (genvar bank = 0; bank < NrVRFBanks; bank++) begin : gen_reg_banks
    for (genvar cut = 0; cut < N_FU; cut++) begin: gen_vrf_slice
      elen_t [NrReadPortsPerBank-1:0] rdata_int;

      for (genvar port = 0; port < NrReadPortsPerBank; port++) begin: gen_rdata_assignment
        assign rdata[bank][port][ELEN*cut +: ELEN] = rdata_int[port];
      end

      vregfile #(
        .NrReadPorts(NrReadPortsPerBank),
        .NrWords    (NrWordsPerBank    ),
        .WordWidth  (ELEN              )
      ) i_vregfile (
        .clk_i     (clk_i                        ),
        .rst_ni    (rst_ni                       ),
        .testmode_i(testmode_i                   ),
        .waddr_i   (waddr[bank]                  ),
        .wdata_i   (wdata[bank][ELEN*cut +: ELEN]),
        .we_i      (we[bank]                     ),
        .wbe_i     (wbe[bank][ELENB*cut +: ELENB]),
        .raddr_i   (raddr[bank]                  ),
        .rdata_o   (rdata_int                    )
      );
        end
      end

      ////////////////
      // Assertions //
      ////////////////

      // Coverage assertion: Count conflicts on the register file banks.
      // A conflict occurs if more than one write request targets the same bank in the same cycle.
      for (genvar bank = 0; bank < NrVRFBanks; bank++) begin : gen_vrf_write_conflict_cov
        cover property (@(posedge clk_i) disable iff (!rst_ni)
      $countones(write_request[bank]) > 1);
      end

      // Same assertion but for read requests. We can have up to 3 read requests per bank
      for (genvar bank = 0; bank < NrVRFBanks; bank++) begin : gen_vrf_read_conflict_cov
        cover property (@(posedge clk_i) disable iff (!rst_ni)
      $countones(read_request[bank]) > 3);
      end

      if (NrReadPorts < 1)
        $error("[spatz_vrf] The number of read ports has to be greater than zero.");

      if (NrWritePorts < 1)
        $error("[spatz_vrf] The number of write ports has to be greater than zero.");

      if (NrReadPorts / NrReadPortsPerBank > NrVRFBanks)
        $error("[spatz_vrf] The number of vregfile banks needs to be increased to handle the number of read ports.");


    endmodule : spatz_vrf

