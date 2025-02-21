// Copyright 2020 OpenHW Group
// Copyright 2020 Datum Technology Corporation
// 
// Licensed under the Solderpad Hardware Licence, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     https://solderpad.org/licenses/
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


`ifndef __UVMA_OBI_COV_MODEL_SV__
`define __UVMA_OBI_COV_MODEL_SV__

   /*
   * Covergroups
   * Decalred at package-level to enable mutliple instances per monitor class (e.g. read/write)
   */
covergroup cg_delay(string name) with function sample(uvma_obi_mon_trn_c trn);
   option.per_instance = 1;
   option.name         = name;

   req_to_gnt: coverpoint (trn.req_to_gnt_delay_cycles) {
      bins dly[] = { [0:3] };
   }
   rready_to_rvalid: coverpoint (trn.rready_to_rvalid_delay_cycles) {
      bins dly[] = { [0:3] };
   }   

   dly_cross: cross req_to_gnt, rready_to_rvalid;
endgroup : cg_delay

/**
 * Component encapsulating Obi functional coverage model.
 */
class uvma_obi_cov_model_c extends uvm_component;
   

   // Objects
   uvma_obi_cfg_c       cfg;
   uvma_obi_cntxt_c     cntxt;
   uvma_obi_mon_trn_c   mon_trn;
   uvma_obi_seq_item_c  seq_item;
   
   // Covergroups 
   cg_delay             wr_dly_cg;
   cg_delay             rd_dly_cg;

   // TLM
   uvm_tlm_analysis_fifo#(uvma_obi_mon_trn_c )  mon_trn_fifo;
   uvm_tlm_analysis_fifo#(uvma_obi_seq_item_c)  seq_item_fifo;
   
   
   `uvm_component_utils_begin(uvma_obi_cov_model_c)
      `uvm_field_object(cfg  , UVM_DEFAULT)
      `uvm_field_object(cntxt, UVM_DEFAULT)
   `uvm_component_utils_end
   
   
   /**
    * Default constructor.
    */
   extern function new(string name="uvma_obi_cov_model", uvm_component parent=null);
   
   /**
    * 1. Ensures cfg & cntxt handles are not null.
    * 2. Builds fifos.
    */
   extern virtual function void build_phase(uvm_phase phase);
   
   /**
    * Forks all sampling loops
    */
   extern virtual task run_phase(uvm_phase phase);
   
   /**
    * TODO Describe sample_cfg
    */
   extern virtual function void sample_cfg();
   
   /**
    * TODO Describe sample_cntxt
    */
   extern virtual function void sample_cntxt();
   
   /**
    * TODO Describe sample_mon_trn
    */
   extern virtual function void sample_mon_trn();
   
   /**
    * TODO Describe sample_seq_item
    */
   extern virtual function void sample_seq_item();
   
endclass : uvma_obi_cov_model_c


`pragma protect begin


function uvma_obi_cov_model_c::new(string name="uvma_obi_cov_model", uvm_component parent=null);
   
   super.new(name, parent);
   
endfunction : new


function void uvma_obi_cov_model_c::build_phase(uvm_phase phase);
   
   super.build_phase(phase);
   
   void'(uvm_config_db#(uvma_obi_cfg_c)::get(this, "", "cfg", cfg));
   if (!cfg) begin
      `uvm_fatal("CFG", "Configuration handle is null")
   end
   
   void'(uvm_config_db#(uvma_obi_cntxt_c)::get(this, "", "cntxt", cntxt));
   if (!cntxt) begin
      `uvm_fatal("CNTXT", "Context handle is null")
   end
   
   mon_trn_fifo  = new("mon_trn_fifo" , this);
   seq_item_fifo = new("seq_item_fifo", this);
   
   if (cfg.enabled && cfg.cov_model_enabled) begin
      if (cfg.read_enabled)
         rd_dly_cg = new("rd_dly_cg");

      if (cfg.write_enabled)
         wr_dly_cg = new("wr_dly_cg");
   end

endfunction : build_phase

task uvma_obi_cov_model_c::run_phase(uvm_phase phase);
   
   super.run_phase(phase);
   
   if (cfg.enabled && cfg.cov_model_enabled) begin
      fork
         // Configuration
         forever begin
            cntxt.sample_cfg_e.wait_trigger();
            sample_cfg();
         end
         
         // Context
         forever begin
            cntxt.sample_cntxt_e.wait_trigger();
            sample_cntxt();
         end
         
         // Monitor transactions
         
         forever begin
            mon_trn_fifo.get(mon_trn);
            sample_mon_trn();
         end
         
         // Sequence items
         forever begin
            seq_item_fifo.get(seq_item);
            sample_seq_item();
         end
      join_none
   end
   
endtask : run_phase


function void uvma_obi_cov_model_c::sample_cfg();
   
   // TODO Implement uvma_obi_cov_model_c::sample_cfg();
   
endfunction : sample_cfg


function void uvma_obi_cov_model_c::sample_cntxt();
   
   // TODO Implement uvma_obi_cov_model_c::sample_cntxt();
   
endfunction : sample_cntxt


function void uvma_obi_cov_model_c::sample_mon_trn();
   
   if (mon_trn.we)
      wr_dly_cg.sample(mon_trn);
   else
      rd_dly_cg.sample(mon_trn);
   
endfunction : sample_mon_trn


function void uvma_obi_cov_model_c::sample_seq_item();
   
   // TODO Implement uvma_obi_cov_model_c::sample_seq_item();
   
endfunction : sample_seq_item


`pragma protect end


`endif // __UVMA_OBI_COV_MODEL_SV__
