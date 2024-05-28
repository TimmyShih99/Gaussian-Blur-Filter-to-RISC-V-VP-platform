#ifndef SOBEL_FILTER_H_
#define SOBEL_FILTER_H_
#include <systemc>
#include <cmath>
#include <iomanip>
using namespace sc_core;

#include <tlm>
#include <tlm_utils/simple_target_socket.h>
#include <algorithm>
#include "filter_def.h"

struct SobelFilter : public sc_module {
  tlm_utils::simple_target_socket<SobelFilter> tsock;

  sc_fifo<unsigned char> i_r;
  sc_fifo<unsigned char> i_g;
  sc_fifo<unsigned char> i_b;
  sc_fifo<int> o_result; // + unsigned

  SC_HAS_PROCESS(SobelFilter);

  SobelFilter(sc_module_name n): 
    sc_module(n), 
    tsock("t_skt"), 
    base_offset(0) 
  {
    tsock.register_b_transport(this, &SobelFilter::blocking_transport);
    SC_THREAD(do_filter);
  }

  ~SobelFilter() {
	}
  const int mask[5][5] = {{1, 4, 7, 4, 1}, {4, 16, 26, 16, 4}, {7, 26, 41, 26, 7}, {4, 16, 26, 16, 4}, {1, 4, 7, 4, 1}}; 
  //int val[MASK_N];
  unsigned int base_offset;

  void do_filter(){   //改這
    cout << "Start Filter::do_filter" << endl;
    { wait(CLOCK_PERIOD, SC_NS); }

    int buffer[25];
    for (unsigned int u = 0 ; u<25; ++u) { // set buffer
		  buffer[u] = 0;
	  }

    while (true) {

      for (unsigned int k = 24 ; k>=5; k-=1) { // store
			  buffer[k] = buffer[k-5];
		  }

      for (unsigned int v = 0; v<5; ++v) { // RGB
        buffer[v] = (i_r.read() + i_g.read() + i_b.read()) / 3;
        //cout << "buffer[v] = " << buffer[v] << endl;
      }
      
      sc_dt::sc_uint<17> val = 0;
      for (unsigned int i = 0; i<5; ++i) { 
        for (unsigned int j = 0; j<5; ++j) { 
          val += buffer[i*5+j]*mask[i][j];
			  }
		  }

      int result = val/273;

      o_result.write(result);
      wait(CLOCK_PERIOD*1, SC_NS);
    }
  }          //到這

  void blocking_transport(tlm::tlm_generic_payload &payload, sc_core::sc_time &delay){
    delay += sc_time(CLOCK_PERIOD*1, SC_NS);
    //cout << "blocking_transport called" <<endl; 
    // unsigned char *mask_ptr = payload.get_byte_enable_ptr();
    // auto len = payload.get_data_length();
    tlm::tlm_command cmd = payload.get_command();
    sc_dt::uint64 addr = payload.get_address();
    unsigned char *data_ptr = payload.get_data_ptr();

    addr -= base_offset;


    // cout << (int)data_ptr[0] << endl;
    // cout << (int)data_ptr[1] << endl;
    // cout << (int)data_ptr[2] << endl;
    word buffer;
    //cout << cmd << endl;
    switch (cmd) {
      case tlm::TLM_READ_COMMAND:
        // cout << "READ" << endl;
        switch (addr) {
          case SOBEL_FILTER_RESULT_ADDR:
            buffer.uint = o_result.read();
            break;
          default:
            std::cerr << "READ Error! SobelFilter::blocking_transport: address 0x"
                      << std::setfill('0') << std::setw(8) << std::hex << addr
                      << std::dec << " is not valid" << std::endl;
          }
        data_ptr[0] = buffer.uc[0];
        data_ptr[1] = buffer.uc[1];
        data_ptr[2] = buffer.uc[2];
        data_ptr[3] = buffer.uc[3];
        //cout << int(buffer.uc[0]) <<", "<< int(buffer.uc[3]) <<" ; ";
        break;

      case tlm::TLM_WRITE_COMMAND:
        // cout << "WRITE" << endl;
        switch (addr) {
          case SOBEL_FILTER_R_ADDR:
            i_r.write(data_ptr[0]);
            i_g.write(data_ptr[1]);
            i_b.write(data_ptr[2]);
            break;
          default:
            std::cerr << "WRITE Error! SobelFilter::blocking_transport: address 0x"
                      << std::setfill('0') << std::setw(8) << std::hex << addr
                      << std::dec << " is not valid" << std::endl;
        }
        break;
      case tlm::TLM_IGNORE_COMMAND:
        payload.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
        return;
      default:
        payload.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
        return;
      }
      payload.set_response_status(tlm::TLM_OK_RESPONSE); // Always OK
  }
};
#endif
