// 
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
// 


`ifndef __UVMA_CLKNRST_BASE_SEQ_SV__
`define __UVMA_CLKNRST_BASE_SEQ_SV__


/**
 * Abstract object from which all other Clock & Reset agent sequences must extend.
 * Subclasses must be run on Clock & Reset sequencer (uvma_clknrst_sqr_c) instance.
 */
class uvma_clknrst_base_seq_c extends uvm_sequence#(
   .REQ(uvma_clknrst_seq_item_c),
   .RSP(uvma_clknrst_seq_item_c)
);
   
   `uvm_object_utils(uvma_clknrst_base_seq_c)
   `uvm_declare_p_sequencer(uvma_clknrst_sqr_c)
   
   
   /**
    * Default constructor.
    */
   extern function new(string name="uvma_clknrst_base_seq");
   
endclass : uvma_clknrst_base_seq_c


function uvma_clknrst_base_seq_c::new(string name="uvma_clknrst_base_seq");
   
   super.new(name);
   
endfunction : new


`endif // __UVMA_CLKNRST_BASE_SEQ_SV__
