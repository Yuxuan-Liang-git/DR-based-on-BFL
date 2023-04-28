#include <sys/time.h>
#include "Filtered_data.h"

#include <iostream>
#include <fstream>

// Include file with properties
int main(int argc, char** argv)
{
  Filtered_data data(off_line_data);  
  data.save(output_data);
  data.get_ekf_data();

  data.save_ekf_data(ekf_data_path);
  data.save_intergrate_pos(pos_path);
  data.save_gps_pos(gps_pos_path);
  data.save_gps_data(gps_data_path);
  data.save_filtered_pos_enu(ekf_enu_data_path);

  
}

