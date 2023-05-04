#include "position_handle.hpp"

int main()
{

    Data_handle data(off_line_data);
    data.data2yaw_rate_const();  //  得到常转向比的数据
    data.MOKFilter();
    
    data.save_vector(data.A_weight,Matrix_output_path);
    // data.save_intergrate_pos(integrate_pos_output_path);
    // data.save_const_coef_pos(const_coef_pos_output_path);

    data.save_const_kf_pos(const_coef_pos_output_path);
    data.save_kf_pos(integrate_pos_output_path);

    data.save_MOKF_pos(MOKF_pos_output_path);

    // data.save_yaw_rate(yaw_rate_data_path);

    // data.show(1,100);
    // data.save(data_output_path);
    // data.get_steer2wheel_dic(steer2wheel_dic_path);


	return 0;
}
