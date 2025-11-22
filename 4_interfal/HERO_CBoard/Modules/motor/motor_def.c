#include "motor_def.h"
#include "PID.h"

Controller_s* create_controller(controller_Init_config_t* _config)
{
    Controller_s* obj = (Controller_s *)malloc(sizeof(Controller_s));
    memset(obj, 0, sizeof(Controller_s));
	obj->motor_setting = _config->setting_config;
    
//	if (_config->setting_config.control_mode == PID_MODEL) {
//		obj->other_speed_pid.pid_config = _config->other_speed_pid_config;
//		obj->other_angle_pid.pid_config = _config->other_position_pid_config;
//		obj->current_pid.pid_config = _config->current_pid_config;
//		obj->speed_pid.pid_config = _config->speed_pid_config;
//		obj->angle_pid.pid_config = _config->position_pid_config;
//	}
    return obj;
}

float controller_clc(Controller_s *obj)
{
//	 if (obj->motor_setting.control_mode == PID_MODEL) {
//		if( obj->motor_setting.close_loop_type >= ANGLE_LOOP) {
//			if ( obj->motor_setting.angle_feedback_source == MOTOR_FEED ) {
//				   PID_Control_Smis(obj->ref_position, obj->fdb_position, &obj->Pos_pid, obj->fdb_speed);
//			       obj->ref_speed = obj->Pos_pid.pid_out;
//			}
//			else {
//				  PID_Control_Smis(obj->ref_position, obj->fdb_position, &obj->OtherPos_pid, obj->fdb_speed);
//		           obj->ref_speed = obj->OtherPos_pid.pid_out; 
//			    }
//		  }
//		if( obj->motor_setting.close_loop_type >= SPEED_LOOP) {
//			if ( obj->motor_setting.feedback_reverse_flag == FEEDBACK_REVERSE )
//				obj->ref_speed *= -1;
//			
//			if ( obj->motor_setting.speed_feedback_source == MOTOR_FEED ) {
//				    PID_Control(obj->ref_speed, obj->fdb_speed, &obj->Speed_pid);
//				  obj->output = obj->Speed_pid.pid_out;
//			}
//			else  {
//				PID_Control(obj->ref_speed, obj->fdb_speed, &obj->OtherSpeed_pid);
//			    obj->output = obj->OtherSpeed_pid.pid_out;
//			}
//			obj->ref_current = obj->output ;
//		  }
//		if( obj->motor_setting.close_loop_type >= CURRENT_LOOP) {
//			PID_Control(obj->ref_current, obj->fdb_current, &obj->Current_pid);
//		    obj->output = obj->Current_pid.pid_out; 
//		 }
//	  }
	  return obj->output;
}
	  
//	 else if(obj->motor_setting.control_mode == ADRC_MODEL){
//		  if(obj->motor_setting.close_loop_type >= ANGLE_LOOP) {
//			obj->adrc_angle_data.prog.ref = obj->ref_position;
//            obj->adrc_angle_data.prog.fdb = obj->fdb_position;
////		   obj->control_setting.angle_feedback_source == OTHER_FEED ? ( obj->adrc_angle_data.prog.fdb = *_config->other_angle_feedback_ptr )
////			  : ( obj->adrc_angle_data.prog.fdb = *_config->angle_feedback_ptr );
//			ADRCFunction(&obj->adrc_angle_data);
//			obj->ref_speed = obj->adrc_angle_data.prog.output;

//		  }
//		  if( obj->motor_setting.close_loop_type >= SPEED_LOOP) {
//			obj->adrc_speed_data.prog.ref = obj->ref_speed;
//			obj->adrc_speed_data.prog.fdb = obj->fdb_speed;  
////			obj->control_setting.speed_feedback_source == OTHER_FEED ? ( obj->adrc_speed_data.prog.fdb = *_config->other_angle_feedback_ptr )
////			  : ( obj->adrc_speed_data.prog.fdb = *_config->angle_feedback_ptr) ;
//            
//			ADRCFunction(&obj->adrc_speed_data);
//			obj->output = obj->adrc_speed_data.prog.output;
//		  }

//	  } else if (obj->motor_setting.control_mode == SMC_MODEL) { // SMC用于单环计算速度
//		   obj->smc_speed_data.ref = obj->ref_speed;
//		   obj->smc_speed_data.fdb = obj->fdb_speed;
//		   SMC_Calc(&obj->smc_speed_data);
//	  			obj->output = obj->smc_speed_data.output;
