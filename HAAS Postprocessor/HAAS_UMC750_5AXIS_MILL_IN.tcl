########################## TCL Event Handlers ##########################
#
#  HAAS_UMC750_5AXIS_MILL_IN.tcl - 5_axis_dual_table
#
#    This is a 5-Axis Milling Machine With
#     Dual Rotary Tables.
#
#  
#
########################################################################



#=============================================================
proc PB_CMD___log_revisions { } {
#=============================================================
# Dummy command to log changes in this post --
#
# 02-26-09 gsl - Initial version
# 09-29-09 gsl - Added custom commands for handling NURBS output
#              - PB_CMD_select_mcs was PB_CMD_mcs_select.
#              - Removed PB_CMD_before_rapid
# 10-27-10 gsl - (PR6429527) Added condition in PB_CMD_init_before_first_tool
#                to protect block auto_tool_change_7 from being
#                obliterated in 3-axis post
# 10-18-2013 levi - Bug fix: Remove PB_CMD_reset_all_motion_variables_to_zero in end of path to support ROTATE UDE.
#                 - Behavior change: No home return in the end of path if there is no tool change in next operation. Home return if it is the last operation. Modified in PB_CMD__check_block_return_to_reference_point.
#                 - Behavior change: Table axis will not return to zero if the tool vector turn to be along Z axis in cycle plane change or in rapid move. Modified in cycle plane change and rapid move.
#                 - Bug fix in general proc: 4th and 5th axis and related mom motion variable should be switched for head head machine when changing the kinematics. Fixed in DPP_GE_COOR_ROT_AUTO3D and  DPP_GE_RESTORE_KINEMATICS.
#                 - Bug fix: Check on "detect work plane change to lower" in cycle plane change.
# 06-04-2014 Allen - PR7143300 fix: check if variable mom_next_oper_has_tool_change and mom_current_oper_is_last_oper_in_program exists for PB_CMD__check_block_return_to_reference_point
# 05-07-2015 ljt   - PR7162261 PR7281995, add PB_CMD_spindle_orient and PB_CMD__check_block_cycle_rapidtoZ, fix PB_CMD__check_block_check_retract_setting, replace spindle_max_rpm with spindle_rpm in MOM_spindle_rpm
# 17-Aug-2015 gsl - Resurrect blocks of new tapping cycles
# 08-18-2015 gsl - Updated PB_CMD_fix_RAPID_SET
# 08-19-2015 gsl - Corrected potential error with PB_CMD_set_cycle_plane
# 08-21-2015 szl - Fix PR7471332:Parse error during machine code simulation if the UDE Operator Message is added.
# 08-21-2015 szl - Enhance the warning message when users set wrong pitch and wrong spindle speed,fix PR7463004
# 09-16-2015 gsl - Merged for NX11 submission
# 09-16-2015 szl - DPP_GE_RESTORE_KINEMATICS is fixed with save_kin_machine_type exist checking, fix PR7383025
# 09-17-2015 szl - Updated PB_CMD_abort_event. Output a warning message in NC output while postprocessor cannot calculate the valid rotary position, fix PR7465721.
# 18-Sep-2015 ljt - Fix lock axis issues: replace obsolete variables with new iks variables in UNLOCK_AXIS and LOCK_AXIS,
#                 - fix PR6961328 in PB_CMD_MOM_lock_axis, comment out reload mom_pos in LOCK_AXIS_MOTION and lock mom_prev_pos in LINEARIZE_LOCK_MOTION
# 12-22-2015 ljt - Updated PB_CMD_spindle_orient. Remove global declaration and account the rotation of feature reference vector for 3axis machine.
# Feb-03-2016 gsl - Clean up DPP_GE_CALCULATE_COOR_ROT_ANGLE
# May-17-2016 gsl - Touchup comments of some commands
# 07-21-2016 szl - Add DPP_GE_UNSET_KINEMATICS: used to unset saved kinematics variables if needed.
# 08-09-2016 lili - Fix ZYX rotation angle calculation issue in DPP_GE_CALCULATE_COOR_ROT_ANGLE. zero resolution was too high.
# 08-15-2016 szl - Update PB_CMD_MOM_rotate: remove original contents and call proc PB_CMD_kin__MOM_rotate.
# 08-30-2016 gsl - Predefine angle & offset in PB_CMD_check_plane_change_for_swiveling to prevent PB syntax check error
# 02-14-2017 szl - 5-axis ncm for drilling operation.
#                  Add proc DPP_GE_GET_NCM_WORK_PLANE_CHANGE_MODE and PB_CMD_coord_rotation_in_operation, update PB_CMD_before_motion, PB_CMD_customize_output_mode,
#                  PB_CMD_reset_output_mode, PB_CMD__check_block_cycle_plane_change_for_swiveling and PB_CMD_check_plane_change_for_swiveling.
# 08-31-2017 Toon VanderKooi PR8958320 add Generic Feature Operation (DPP_GE_DETECT_HOLE_CUTTING_OPERATION)
# 10-30-2017 cj  - Fix PR9001883: Add tool tip in calculation of mom_pos in function DPP_GE_COOR_ROT_AUTO3D.
# 04-02-2018 ljt - PR9080157, take account of rotary axis direction in DPP_GE_COOR_ROT_AUTO3D
# 11-Jul-2018 gsl - Enhanced DPP_GE_SAVE_KINEMATICS & DPP_GE_RESTORE_KINEMATICS to accommodate linked post
#                 - Removed unused PB_CMD_restore_kinematics & PB_CMD_save_kinematics
# 24-Jul-2018 gsl - Enhanced check condition for axis_limit_action in ROTARY_AXIS_RETRACT
# 23-Oct-2018 gsl - Fixed problem in DPP_GE_SAVE_KINEMATICS where ::save_mom_kin_machine_type is not saved in linked post setting.
# 26-Nov-2018 gsl - Devize PB_CMD_turn_on_read_ahead to centrally control read ahead
# 27-Nov-2018 gsl - Call PB_CMD__convert_point in place of MOM_convert_point
# 02-Aug-2019 gsl - Added commands for handling patterned path subprogrom and transition path output
#
}



  set cam_post_dir [MOM_ask_env_var UGII_CAM_POST_DIR]

  set mom_sys_this_post_dir  "[file dirname [info script]]"
  set mom_sys_this_post_name "[file rootname [file tail [info script]]]"


  if { ![info exists mom_sys_post_initialized] } {

     if { ![info exists mom_sys_ugpost_base_initialized] } {
        source ${cam_post_dir}ugpost_base.tcl
        set mom_sys_ugpost_base_initialized 1
     }


     set mom_sys_debug_mode OFF


     if { ![info exists env(PB_SUPPRESS_UGPOST_DEBUG)] } {
        set env(PB_SUPPRESS_UGPOST_DEBUG) 0
     }

     if { $env(PB_SUPPRESS_UGPOST_DEBUG) } {
        set mom_sys_debug_mode OFF
     }

     if { ![string compare $mom_sys_debug_mode "OFF"] } {

        proc MOM_before_each_add_var {} {}
        proc MOM_before_each_event   {} {}
        proc MOM_before_load_address {} {}
        proc MOM_end_debug {} {}

     } else {

        set cam_debug_dir [MOM_ask_env_var UGII_CAM_DEBUG_DIR]
        source ${cam_debug_dir}mom_review.tcl
     }


   ####  Listing File variables
     set mom_sys_list_output                       "OFF"
     set mom_sys_header_output                     "OFF"
     set mom_sys_list_file_rows                    "40"
     set mom_sys_list_file_columns                 "30"
     set mom_sys_warning_output                    "OFF"
     set mom_sys_warning_output_option             "FILE"
     set mom_sys_group_output                      "OFF"
     set mom_sys_list_file_suffix                  "lpt"
     set mom_sys_output_file_suffix                "H"
     set mom_sys_commentary_output                 "ON"
     set mom_sys_commentary_list                   "x y z 4axis 5axis feed speed"
     set mom_sys_output_transition_path            "0"
     set mom_sys_post_output_subprogram_enabled    "0"
     set mom_sys_pb_link_var_mode                  "OFF"


     if { [string match "OFF" $mom_sys_warning_output] } {
        catch { rename MOM__util_print ugpost_MOM__util_print }
        proc MOM__util_print { args } {}
     }


     MOM_set_debug_mode $mom_sys_debug_mode


     if { [string match "OFF" $mom_sys_warning_output] } {
        catch { rename MOM__util_print "" }
        catch { rename ugpost_MOM__util_print MOM__util_print }
     }


   #=============================================================
   proc MOM_before_output { } {
   #=============================================================
   # This command is executed just before every NC block is
   # to be output to a file.
   #
   # - Never overload this command!
   # - Any customization should be done in PB_CMD_before_output!
   #

      if { [llength [info commands PB_CMD_kin_before_output]] &&\
           [llength [info commands PB_CMD_before_output]] } {

         PB_CMD_kin_before_output
      }

     # Write output buffer to the listing file with warnings
      global mom_sys_list_output
      if { [string match "ON" $mom_sys_list_output] } {
         LIST_FILE
      } else {
         global tape_bytes mom_o_buffer
         if { ![info exists tape_bytes] } {
            set tape_bytes [string length $mom_o_buffer]
         } else {
            incr tape_bytes [string length $mom_o_buffer]
         }
      }
   }


     if { [string match "OFF" [MOM_ask_env_var UGII_CAM_POST_LINK_VAR_MODE]] } {
        set mom_sys_link_var_mode                     "OFF"
     } else {
        set mom_sys_link_var_mode                     "$mom_sys_pb_link_var_mode"
     }


     set mom_sys_control_out                       "("
     set mom_sys_control_in                        ")"


    # Retain UDE handlers of ugpost_base
     foreach ude_handler { MOM_insert \
                           MOM_operator_message \
                           MOM_opskip_off \
                           MOM_opskip_on \
                           MOM_pprint \
                           MOM_text \
                         } \
     {
        if { [llength [info commands $ude_handler]] &&\
            ![llength [info commands ugpost_${ude_handler}]] } {
           rename $ude_handler ugpost_${ude_handler}
        }
     }


     set mom_sys_post_initialized 1
  }


  set mom_sys_use_default_unit_fragment         "ON"
  set mom_sys_alt_unit_post_name                "HAAS_UMC750_5AXIS_MILL_IN__MM.pui"


########## SYSTEM VARIABLE DECLARATIONS ##############
  set mom_sys_rapid_code                        "0"
  set mom_sys_linear_code                       "1"
  set mom_sys_circle_code(CLW)                  "2"
  set mom_sys_circle_code(CCLW)                 "3"
  set mom_sys_delay_code(SECONDS)               "4"
  set mom_sys_delay_code(REVOLUTIONS)           "4"
  set mom_sys_cutcom_plane_code(XY)             "17"
  set mom_sys_cutcom_plane_code(ZX)             "18"
  set mom_sys_cutcom_plane_code(XZ)             "18"
  set mom_sys_cutcom_plane_code(YZ)             "19"
  set mom_sys_cutcom_plane_code(ZY)             "19"
  set mom_sys_cutcom_code(OFF)                  "40"
  set mom_sys_cutcom_code(LEFT)                 "41"
  set mom_sys_cutcom_code(RIGHT)                "42"
  set mom_sys_adjust_code                       "43"
  set mom_sys_adjust_code_minus                 "44"
  set mom_sys_adjust_cancel_code                "49"
  set mom_sys_unit_code(IN)                     "20"
  set mom_sys_unit_code(MM)                     "21"
  set mom_sys_cycle_start_code                  "79"
  set mom_sys_cycle_off                         "80"
  set mom_sys_cycle_drill_code                  "81"
  set mom_sys_cycle_drill_dwell_code            "82"
  set mom_sys_cycle_drill_deep_code             "83"
  set mom_sys_cycle_drill_break_chip_code       "73"
  set mom_sys_cycle_tap_code                    "84"
  set mom_sys_cycle_tap_float_code              "84"
  set mom_sys_cycle_tap_deep_code               "84"
  set mom_sys_cycle_tap_break_chip_code         "84"
  set mom_sys_cycle_bore_code                   "85"
  set mom_sys_cycle_bore_drag_code              "86"
  set mom_sys_cycle_bore_no_drag_code           "76"
  set mom_sys_cycle_bore_dwell_code             "89"
  set mom_sys_cycle_bore_manual_code            "88"
  set mom_sys_cycle_bore_back_code              "87"
  set mom_sys_cycle_bore_manual_dwell_code      "88"
  set mom_sys_output_code(ABSOLUTE)             "90"
  set mom_sys_output_code(INCREMENTAL)          "91"
  set mom_sys_cycle_ret_code(AUTO)              "98"
  set mom_sys_cycle_ret_code(MANUAL)            "99"
  set mom_sys_reset_code                        "92"
  set mom_sys_feed_rate_mode_code(IPM)          "94"
  set mom_sys_feed_rate_mode_code(IPR)          "95"
  set mom_sys_feed_rate_mode_code(FRN)          "93"
  set mom_sys_spindle_mode_code(SFM)            "96"
  set mom_sys_spindle_mode_code(RPM)            "97"
  set mom_sys_return_code                       "28"
  set mom_sys_feed_rate_mode_code(MMPM)         "94"
  set mom_sys_feed_rate_mode_code(MMPR)         "95"
  set mom_sys_feed_rate_mode_code(DPM)          "94"
  set mom_sys_program_stop_code                 "0"
  set mom_sys_optional_stop_code                "1"
  set mom_sys_end_of_program_code               "2"
  set mom_sys_spindle_direction_code(CLW)       "3"
  set mom_sys_spindle_direction_code(CCLW)      "4"
  set mom_sys_spindle_direction_code(OFF)       "5"
  set mom_sys_tool_change_code                  "6"
  set mom_sys_coolant_code(ON)                  "8"
  set mom_sys_coolant_code(FLOOD)               "8"
  set mom_sys_coolant_code(MIST)                "7"
  set mom_sys_coolant_code(THRU)                "26"
  set mom_sys_coolant_code(TAP)                 "73"
  set mom_sys_coolant_code(OFF)                 "9"
  set mom_sys_rewind_code                       "30"
  set mom_sys_4th_axis_has_limits               "1"
  set mom_sys_5th_axis_has_limits               "1"
  set mom_sys_sim_cycle_drill                   "0"
  set mom_sys_sim_cycle_drill_dwell             "0"
  set mom_sys_sim_cycle_drill_deep              "0"
  set mom_sys_sim_cycle_drill_break_chip        "0"
  set mom_sys_sim_cycle_tap                     "1"
  set mom_sys_sim_cycle_tap_float               "1"
  set mom_sys_sim_cycle_tap_deep                "1"
  set mom_sys_sim_cycle_tap_break_chip          "1"
  set mom_sys_sim_cycle_bore                    "0"
  set mom_sys_sim_cycle_bore_drag               "0"
  set mom_sys_sim_cycle_bore_nodrag             "0"
  set mom_sys_sim_cycle_bore_manual             "0"
  set mom_sys_sim_cycle_bore_dwell              "0"
  set mom_sys_sim_cycle_bore_manual_dwell       "0"
  set mom_sys_sim_cycle_bore_back               "0"
  set mom_sys_cir_vector                        "Vector - Arc Start to Center"
  set mom_sys_spindle_ranges                    "9"
  set mom_sys_rewind_stop_code                  "\#"
  set mom_sys_home_pos(0)                       "0"
  set mom_sys_home_pos(1)                       "0"
  set mom_sys_home_pos(2)                       "0"
  set mom_sys_zero                              "0"
  set mom_sys_opskip_block_leader               "/"
  set mom_sys_seqnum_start                      "5"
  set mom_sys_seqnum_incr                       "5"
  set mom_sys_seqnum_freq                       "1"
  set mom_sys_seqnum_max                        "99999"
  set mom_sys_lathe_x_double                    "1"
  set mom_sys_lathe_i_double                    "1"
  set mom_sys_lathe_y_double                    "1"
  set mom_sys_lathe_j_double                    "1"
  set mom_sys_lathe_x_factor                    "1"
  set mom_sys_lathe_y_factor                    "1"
  set mom_sys_lathe_z_factor                    "1"
  set mom_sys_lathe_i_factor                    "1"
  set mom_sys_lathe_j_factor                    "1"
  set mom_sys_lathe_k_factor                    "1"
  set mom_sys_leader(N)                         "N"
  set mom_sys_leader(X)                         "X"
  set mom_sys_leader(Y)                         "Y"
  set mom_sys_leader(Z)                         "Z"
  set mom_sys_leader(fourth_axis)               "B"
  set mom_sys_leader(fifth_axis)                "C"
  set mom_sys_contour_feed_mode(LINEAR)         "IPM"
  set mom_sys_rapid_feed_mode(LINEAR)           "IPM"
  set mom_sys_cycle_feed_mode                   "IPM"
  set mom_sys_feed_param(IPM,format)            "Feed_IPM"
  set mom_sys_feed_param(IPR,format)            "Feed_IPR"
  set mom_sys_feed_param(FRN,format)            "Feed_INV"
  set mom_sys_vnc_rapid_dogleg                  "1"
  set mom_sys_prev_mach_head                    ""
  set mom_sys_curr_mach_head                    ""
  set mom_sys_contour_feed_mode(ROTARY)         "IPM"
  set mom_sys_contour_feed_mode(LINEAR_ROTARY)  "IPM"
  set mom_sys_feed_param(DPM,format)            "Feed_DPM"
  set mom_sys_rapid_feed_mode(ROTARY)           "IPM"
  set mom_sys_rapid_feed_mode(LINEAR_ROTARY)    "IPM"
  set mom_sys_feed_param(MMPM,format)           "Feed_MMPM"
  set mom_sys_feed_param(MMPR,format)           "Feed_MMPR"
  set mom_sys_advanced_turbo_output             "0"
  set mom_sys_linearization_method              "angle"
  set mom_sys_tool_number_max                   "32"
  set mom_sys_tool_number_min                   "1"
  set mom_sys_post_description                  "This is a 5-Axis Milling Machine With\n\
                                                  Dual Rotary Tables."
  set mom_sys_word_separator                    " "
  set mom_sys_end_of_block                      ""
  set mom_sys_ugpadvkins_used                   "0"
  set mom_sys_post_builder_version              "1899"

####### KINEMATIC VARIABLE DECLARATIONS ##############
  set mom_kin_4th_axis_ang_offset               "0.0"
  set mom_kin_4th_axis_center_offset(0)         "0.0"
  set mom_kin_4th_axis_center_offset(1)         "0.0"
  set mom_kin_4th_axis_center_offset(2)         "0.0"
  set mom_kin_4th_axis_direction                "MAGNITUDE_DETERMINES_DIRECTION"
  set mom_kin_4th_axis_incr_switch              "OFF"
  set mom_kin_4th_axis_leader                   "B"
  set mom_kin_4th_axis_limit_action             "Warning"
  set mom_kin_4th_axis_max_limit                "120.0"
  set mom_kin_4th_axis_min_incr                 "0.001"
  set mom_kin_4th_axis_min_limit                "-35.0"
  set mom_kin_4th_axis_plane                    "ZX"
  set mom_kin_4th_axis_point(0)                 "0.0"
  set mom_kin_4th_axis_point(1)                 "0.0"
  set mom_kin_4th_axis_point(2)                 "0.0"
  set mom_kin_4th_axis_rotation                 "standard"
  set mom_kin_4th_axis_type                     "Table"
  set mom_kin_4th_axis_vector(0)                "0"
  set mom_kin_4th_axis_vector(1)                "1"
  set mom_kin_4th_axis_vector(2)                "0"
  set mom_kin_4th_axis_zero                     "0.0"
  set mom_kin_5th_axis_ang_offset               "0.0"
  set mom_kin_5th_axis_center_offset(0)         "0.0"
  set mom_kin_5th_axis_center_offset(1)         "0.0"
  set mom_kin_5th_axis_center_offset(2)         "0.0"
  set mom_kin_5th_axis_direction                "MAGNITUDE_DETERMINES_DIRECTION"
  set mom_kin_5th_axis_incr_switch              "OFF"
  set mom_kin_5th_axis_leader                   "C"
  set mom_kin_5th_axis_limit_action             "Warning"
  set mom_kin_5th_axis_max_limit                "999999999.999"
  set mom_kin_5th_axis_min_incr                 "0.001"
  set mom_kin_5th_axis_min_limit                "-999999999.999"
  set mom_kin_5th_axis_plane                    "XY"
  set mom_kin_5th_axis_point(0)                 "0.0"
  set mom_kin_5th_axis_point(1)                 "0.0"
  set mom_kin_5th_axis_point(2)                 "0.0"
  set mom_kin_5th_axis_rotation                 "standard"
  set mom_kin_5th_axis_type                     "Table"
  set mom_kin_5th_axis_vector(0)                "0"
  set mom_kin_5th_axis_vector(1)                "0"
  set mom_kin_5th_axis_vector(2)                "1"
  set mom_kin_5th_axis_zero                     "0.0"
  set mom_kin_arc_output_mode                   "FULL_CIRCLE"
  set mom_kin_arc_valid_plane                   "XY"
  set mom_kin_clamp_time                        "2.0"
  set mom_kin_cycle_plane_change_per_axis       "1"
  set mom_kin_cycle_plane_change_to_lower       "1"
  set mom_kin_flush_time                        "2.0"
  set mom_kin_linearization_flag                "1"
  set mom_kin_linearization_tol                 "0.001"
  set mom_kin_machine_resolution                ".0001"
  set mom_kin_machine_type                      "5_axis_dual_table"
  set mom_kin_machine_zero_offset(0)            "0.0"
  set mom_kin_machine_zero_offset(1)            "0.0"
  set mom_kin_machine_zero_offset(2)            "0.0"
  set mom_kin_max_arc_radius                    "9999.9999"
  set mom_kin_max_dpm                           "1000"
  set mom_kin_max_fpm                           "600"
  set mom_kin_max_fpr                           "100"
  set mom_kin_max_frn                           "1000"
  set mom_kin_min_arc_length                    "0.01"
  set mom_kin_min_arc_radius                    "0.0001"
  set mom_kin_min_dpm                           "0.0"
  set mom_kin_min_fpm                           "0.1"
  set mom_kin_min_fpr                           "0.01"
  set mom_kin_min_frn                           "0.01"
  set mom_kin_output_unit                       "IN"
  set mom_kin_pivot_gauge_offset                "0.0"
  set mom_kin_pivot_guage_offset                ""
  set mom_kin_post_data_unit                    "IN"
  set mom_kin_rapid_feed_rate                   "400"
  set mom_kin_retract_distance                  "10"
  set mom_kin_rotary_axis_method                "PREVIOUS"
  set mom_kin_spindle_axis(0)                   "0.0"
  set mom_kin_spindle_axis(1)                   "0.0"
  set mom_kin_spindle_axis(2)                   "1.0"
  set mom_kin_tool_change_time                  "12.0"
  set mom_kin_x_axis_limit                      "30"
  set mom_kin_y_axis_limit                      "20"
  set mom_kin_z_axis_limit                      "20"




if [llength [info commands MOM_SYS_do_template]] {
   if [llength [info commands MOM_do_template]] {
      rename MOM_do_template ""
   }
   rename MOM_SYS_do_template MOM_do_template
}




#=============================================================
proc MOM_start_of_program { } {
#=============================================================
  global mom_logname mom_date is_from
  global mom_coolant_status mom_cutcom_status
  global mom_clamp_status mom_cycle_status
  global mom_spindle_status mom_cutcom_plane pb_start_of_program_flag
  global mom_cutcom_adjust_register mom_tool_adjust_register
  global mom_tool_length_adjust_register mom_length_comp_register
  global mom_flush_register mom_wire_cutcom_adjust_register
  global mom_wire_cutcom_status

    set pb_start_of_program_flag 0
    set mom_coolant_status UNDEFINED
    set mom_cutcom_status  UNDEFINED
    set mom_clamp_status   UNDEFINED
    set mom_cycle_status   UNDEFINED
    set mom_spindle_status UNDEFINED
    set mom_cutcom_plane   UNDEFINED
    set mom_wire_cutcom_status  UNDEFINED

    catch {unset mom_cutcom_adjust_register}
    catch {unset mom_tool_adjust_register}
    catch {unset mom_tool_length_adjust_register}
    catch {unset mom_length_comp_register}
    catch {unset mom_flush_register}
    catch {unset mom_wire_cutcom_adjust_register}

    set is_from ""

    catch { OPEN_files } ;# Open warning and listing files
    LIST_FILE_HEADER     ;# List header in commentary listing



  global mom_sys_post_initialized
  if { $mom_sys_post_initialized > 1 } { return }


  set ::mom_sys_start_program_clock_seconds [clock seconds]

   # Load parameters for alternate output units
    PB_load_alternate_unit_settings
    rename PB_load_alternate_unit_settings ""


#************
uplevel #0 {


#=============================================================
proc MOM_sync { } {
#=============================================================
  if [llength [info commands PB_CMD_kin_handle_sync_event] ] {
    PB_CMD_kin_handle_sync_event
  }
}


#=============================================================
proc MOM_set_csys { } {
#=============================================================
  if [llength [info commands PB_CMD_kin_set_csys] ] {
    PB_CMD_kin_set_csys
  }
}


#=============================================================
proc MOM_msys { } {
#=============================================================
}


#=============================================================
proc MOM_end_of_program { } {
#=============================================================
  global mom_program_aborted mom_event_error

   MOM_force Once G_motion G_offset Z
   MOM_do_template return_home_z

   MOM_do_template go_home_xybc

   if { [PB_CMD__check_block_output_unclamp_codes_end_of_program] } {
      MOM_force Once M_clamp_5th Text
      MOM_do_template fifth_axis_unclamp
   }

   if { [PB_CMD__check_block_output_unclamp_codes_end_of_program] } {
      MOM_force Once M_clamp_4th Text
      MOM_do_template fourth_axis_unclamp
   }

   MOM_force Once G_mode G fourth_axis fifth_axis
   MOM_do_template return_home_bc

   MOM_force Once M_clamp_5th Text
   MOM_do_template fifth_axis_clamp

   MOM_force Once M_clamp_4th Text
   MOM_do_template fourth_axis_clamp

   MOM_force Once G_mode
   MOM_do_template absolute_mode

   MOM_force Once M
   MOM_do_template rewind_program
   MOM_set_seq_off

   MOM_force Once Text
   MOM_do_template rewind_stop_code
   PB_CMD_end_of_program

  # Write tool list with time in commentary data
   LIST_FILE_TRAILER

  # Close warning and listing files
   CLOSE_files

   if [CMD_EXIST PB_CMD_kin_end_of_program] {
      PB_CMD_kin_end_of_program
   }
}


  incr mom_sys_post_initialized


} ;# uplevel
#***********


}


#=============================================================
proc PB_init_new_iks { } {
#=============================================================
  global mom_kin_iks_usage mom_kin_spindle_axis
  global mom_kin_4th_axis_vector mom_kin_5th_axis_vector


   set mom_kin_iks_usage 1

  # Override spindle axis vector defined in PB_CMD_init_rotary
   set mom_kin_spindle_axis(0)  0.0
   set mom_kin_spindle_axis(1)  0.0
   set mom_kin_spindle_axis(2)  1.0

  # Unitize vectors
   foreach i { 0 1 2 } {
      set vec($i) $mom_kin_spindle_axis($i)
   }
   VEC3_unitize vec mom_kin_spindle_axis

   foreach i { 0 1 2 } {
      set vec($i) $mom_kin_4th_axis_vector($i)
   }
   VEC3_unitize vec mom_kin_4th_axis_vector

   foreach i { 0 1 2 } {
      set vec($i) $mom_kin_5th_axis_vector($i)
   }
   VEC3_unitize vec mom_kin_5th_axis_vector

  # Reload kinematics
   MOM_reload_kinematics
}


#=============================================================
proc PB_DELAY_TIME_SET { } {
#=============================================================
  global mom_sys_delay_param mom_delay_value
  global mom_delay_revs mom_delay_mode delay_time

  # Post Builder provided format for the current mode:
   if { [info exists mom_sys_delay_param(${mom_delay_mode},format)] != 0 } {
      MOM_set_address_format dwell $mom_sys_delay_param(${mom_delay_mode},format)
   }

   switch $mom_delay_mode {
      SECONDS { set delay_time $mom_delay_value }
      default { set delay_time $mom_delay_revs  }
   }
}


#=============================================================
proc MOM_before_motion { } {
#=============================================================
  global mom_motion_event mom_motion_type

   FEEDRATE_SET

   switch $mom_motion_type {
      ENGAGE   { PB_engage_move }
      APPROACH { PB_approach_move }
      FIRSTCUT { catch {PB_first_cut} }
      RETRACT  { PB_retract_move }
      RETURN   { catch {PB_return_move} }
      default  {}
   }

   if { [llength [info commands PB_CMD_kin_before_motion] ] } { PB_CMD_kin_before_motion }
   if { [llength [info commands PB_CMD_before_motion] ] }     { PB_CMD_before_motion }
}


#=============================================================
proc MOM_start_of_group { } {
#=============================================================
  global mom_sys_group_output mom_group_name group_level ptp_file_name
  global mom_sequence_number mom_sequence_increment mom_sequence_frequency
  global mom_sys_ptp_output pb_start_of_program_flag

   if { ![hiset group_level] } {
      set group_level 0
      return
   }

   if { [hiset mom_sys_group_output] } {
      if { ![string compare $mom_sys_group_output "OFF"] } {
         set group_level 0
         return
      }
   }

   if { [hiset group_level] } {
      incr group_level
   } else {
      set group_level 1
   }

   if { $group_level > 1 } {
      return
   }

   SEQNO_RESET ; #<4133654>
   MOM_reset_sequence $mom_sequence_number $mom_sequence_increment $mom_sequence_frequency

   if { [info exists ptp_file_name] } {
      MOM_close_output_file $ptp_file_name
      MOM_start_of_program
      if { ![string compare $mom_sys_ptp_output "ON"] } {
         MOM_open_output_file $ptp_file_name
      }
   } else {
      MOM_start_of_program
   }

   PB_start_of_program
   set pb_start_of_program_flag 1
}


#=============================================================
proc MOM_machine_mode { } {
#=============================================================
  global pb_start_of_program_flag
  global mom_operation_name mom_sys_change_mach_operation_name

   set mom_sys_change_mach_operation_name $mom_operation_name

   if { $pb_start_of_program_flag == 0 } {
      PB_start_of_program
      set pb_start_of_program_flag 1
   }

  # Reload post for simple mill-turn
   if { [llength [info commands PB_machine_mode]] } {
      if { [catch {PB_machine_mode} res] } {
         CATCH_WARNING "$res"
      }
   }
}


#=============================================================
proc PB_FORCE { option args } {
#=============================================================
   set adds [join $args]
   if { [info exists option] && [llength $adds] } {
      lappend cmd MOM_force
      lappend cmd $option
      lappend cmd [join $adds]
      eval [join $cmd]
   }
}


#=============================================================
proc PB_SET_RAPID_MOD { mod_list blk_list ADDR NEW_MOD_LIST } {
#=============================================================
  upvar $ADDR addr
  upvar $NEW_MOD_LIST new_mod_list
  global mom_cycle_spindle_axis traverse_axis1 traverse_axis2


   set new_mod_list [list]

   foreach mod $mod_list {
      switch $mod {
         "rapid1" {
            set elem $addr($traverse_axis1)
            if { [lsearch $blk_list $elem] >= 0 } {
               lappend new_mod_list $elem
            }
         }
         "rapid2" {
            set elem $addr($traverse_axis2)
            if { [lsearch $blk_list $elem] >= 0 } {
               lappend new_mod_list $elem
            }
         }
         "rapid3" {
            set elem $addr($mom_cycle_spindle_axis)
            if { [lsearch $blk_list $elem] >= 0 } {
               lappend new_mod_list $elem
            }
         }
         default {
            set elem $mod
            if { [lsearch $blk_list $elem] >= 0 } {
               lappend new_mod_list $elem
            }
         }
      }
   }
}


########################
# Redefine FEEDRATE_SET
########################
if { [llength [info commands ugpost_FEEDRATE_SET] ] } {
   rename ugpost_FEEDRATE_SET ""
}

if { [llength [info commands FEEDRATE_SET] ] } {
   rename FEEDRATE_SET ugpost_FEEDRATE_SET
} else {
   proc ugpost_FEEDRATE_SET {} {}
}


#=============================================================
proc FEEDRATE_SET { } {
#=============================================================
   if { [llength [info commands PB_CMD_kin_feedrate_set] ] } {
      PB_CMD_kin_feedrate_set
   } else {
      ugpost_FEEDRATE_SET
   }
}


############## EVENT HANDLING SECTION ################


#=============================================================
proc MOM_auxfun { } {
#=============================================================
   MOM_do_template auxfun
}


#=============================================================
proc MOM_bore { } {
#=============================================================
  global cycle_name
  global cycle_init_flag

   set cycle_init_flag TRUE
   set cycle_name BORE
   CYCLE_SET
}


#=============================================================
proc MOM_bore_move { } {
#=============================================================
   global cycle_init_flag


   ABORT_EVENT_CHECK

   PB_CMD_set_cycle_plane
   PB_CMD_force_cycle_parameters

   MOM_do_template cycle_bore

   if { [PB_CMD__check_block_output_cycle_retract] } {
      MOM_do_template cycle_return
   }
   set cycle_init_flag FALSE
}


#=============================================================
proc MOM_bore_back { } {
#=============================================================
  global cycle_name
  global cycle_init_flag

   set cycle_init_flag TRUE
   set cycle_name BORE_BACK
   CYCLE_SET
}


#=============================================================
proc MOM_bore_back_move { } {
#=============================================================
   global cycle_init_flag


   ABORT_EVENT_CHECK

   PB_CMD_set_cycle_plane
   PB_CMD_force_cycle_parameters

   MOM_do_template cycle_bore_back

   if { [PB_CMD__check_block_output_cycle_retract] } {
      MOM_do_template cycle_return
   }
   set cycle_init_flag FALSE
}


#=============================================================
proc MOM_bore_drag { } {
#=============================================================
  global cycle_name
  global cycle_init_flag

   set cycle_init_flag TRUE
   set cycle_name BORE_DRAG
   CYCLE_SET
}


#=============================================================
proc MOM_bore_drag_move { } {
#=============================================================
   global cycle_init_flag


   ABORT_EVENT_CHECK

   PB_CMD_set_cycle_plane
   PB_CMD_force_cycle_parameters

   MOM_do_template cycle_bore_drag

   if { [PB_CMD__check_block_output_cycle_retract] } {
      MOM_do_template cycle_return
   }
   set cycle_init_flag FALSE
}


#=============================================================
proc MOM_bore_dwell { } {
#=============================================================
  global cycle_name
  global cycle_init_flag

   set cycle_init_flag TRUE
   set cycle_name BORE_DWELL
   CYCLE_SET
}


#=============================================================
proc MOM_bore_dwell_move { } {
#=============================================================
   global cycle_init_flag


   ABORT_EVENT_CHECK

   PB_CMD_set_cycle_plane
   PB_CMD_force_cycle_parameters

   MOM_do_template cycle_bore_dwell

   if { [PB_CMD__check_block_output_cycle_retract] } {
      MOM_do_template cycle_return
   }
   set cycle_init_flag FALSE
}


#=============================================================
proc MOM_bore_manual { } {
#=============================================================
  global cycle_name
  global cycle_init_flag

   set cycle_init_flag TRUE
   set cycle_name BORE_MANUAL
   CYCLE_SET
}


#=============================================================
proc MOM_bore_manual_move { } {
#=============================================================
   global cycle_init_flag


   ABORT_EVENT_CHECK

   PB_CMD_set_cycle_plane
   PB_CMD_force_cycle_parameters

   MOM_do_template cycle_bore_manual

   if { [PB_CMD__check_block_output_cycle_retract] } {
      MOM_do_template cycle_return
   }
   set cycle_init_flag FALSE
}


#=============================================================
proc MOM_bore_manual_dwell { } {
#=============================================================
  global cycle_name
  global cycle_init_flag

   set cycle_init_flag TRUE
   set cycle_name BORE_MANUAL_DWELL
   CYCLE_SET
}


#=============================================================
proc MOM_bore_manual_dwell_move { } {
#=============================================================
   global cycle_init_flag


   ABORT_EVENT_CHECK

   PB_CMD_set_cycle_plane
   PB_CMD_force_cycle_parameters

   MOM_do_template cycle_bore_manual_dwell

   if { [PB_CMD__check_block_output_cycle_retract] } {
      MOM_do_template cycle_return
   }
   set cycle_init_flag FALSE
}


#=============================================================
proc MOM_bore_no_drag { } {
#=============================================================
  global cycle_name
  global cycle_init_flag

   set cycle_init_flag TRUE
   set cycle_name BORE_NO_DRAG
   CYCLE_SET
}


#=============================================================
proc MOM_bore_no_drag_move { } {
#=============================================================
   global cycle_init_flag


   ABORT_EVENT_CHECK

   PB_CMD_set_cycle_plane
   PB_CMD_force_cycle_parameters

   MOM_do_template cycle_bore_no_drag

   if { [PB_CMD__check_block_output_cycle_retract] } {
      MOM_do_template cycle_return
   }
   set cycle_init_flag FALSE
}


#=============================================================
proc MOM_circular_move { } {
#=============================================================
   ABORT_EVENT_CHECK

   CIRCLE_SET
   PB_CMD_cutcom_setting

   if { [PB_CMD__check_block_mill_default_condition] } {
      MOM_force Once G_motion X Y I J
      MOM_do_template circular_move
   }
}


#=============================================================
proc MOM_clamp { } {
#=============================================================
   global mom_clamp_axis
   global mom_clamp_status
   global mom_clamp_text
   PB_CMD_MOM_clamp
}


#=============================================================
proc MOM_coolant_off { } {
#=============================================================
   COOLANT_SET

   MOM_do_template coolant_off
}


#=============================================================
proc MOM_coolant_on { } {
#=============================================================
   COOLANT_SET
}


#=============================================================
proc MOM_cutcom_on { } {
#=============================================================
   CUTCOM_SET

   global mom_cutcom_adjust_register

   if { [info exists mom_cutcom_adjust_register] } {
      set cutcom_register_min 1
      set cutcom_register_max 99

      if { $mom_cutcom_adjust_register < $cutcom_register_min ||\
           $mom_cutcom_adjust_register > $cutcom_register_max } {

         CATCH_WARNING "CUTCOM register $mom_cutcom_adjust_register must be within the range between 1 and 99"

         unset mom_cutcom_adjust_register
      }
   }
}


#=============================================================
proc MOM_cutcom_off { } {
#=============================================================
   CUTCOM_SET
}


#=============================================================
proc MOM_cycle_off { } {
#=============================================================
   PB_CMD_reset_force_cycle_parameters
}


#=============================================================
proc MOM_cycle_plane_change { } {
#=============================================================
  global cycle_init_flag
  global mom_cycle_tool_axis_change
  global mom_cycle_clearance_plane_change

   set cycle_init_flag TRUE
   PB_CMD_cycle_clearance_plane_change
   set cycle_init_flag FALSE
}


#=============================================================
proc MOM_delay { } {
#=============================================================
   PB_DELAY_TIME_SET

   MOM_do_template delay
}


#=============================================================
proc MOM_drill { } {
#=============================================================
  global cycle_name
  global cycle_init_flag

   set cycle_init_flag TRUE
   set cycle_name DRILL
   CYCLE_SET
}


#=============================================================
proc MOM_drill_move { } {
#=============================================================
   global cycle_init_flag


   ABORT_EVENT_CHECK

   PB_CMD_set_cycle_plane
   PB_CMD_force_cycle_parameters

   MOM_do_template cycle_drill

   if { [PB_CMD__check_block_output_cycle_retract] } {
      MOM_do_template cycle_return
   }
   set cycle_init_flag FALSE
}


#=============================================================
proc MOM_drill_break_chip { } {
#=============================================================
  global cycle_name
  global cycle_init_flag

   set cycle_init_flag TRUE
   set cycle_name DRILL_BREAK_CHIP
   CYCLE_SET
}


#=============================================================
proc MOM_drill_break_chip_move { } {
#=============================================================
   global cycle_init_flag


   ABORT_EVENT_CHECK

   PB_CMD_set_cycle_plane
   PB_CMD_force_cycle_parameters
   PB_CMD_remove_Q0

   MOM_do_template cycle_drill_break_chip

   if { [PB_CMD__check_block_output_cycle_retract] } {
      MOM_do_template cycle_return
   }
   set cycle_init_flag FALSE
}


#=============================================================
proc MOM_drill_deep { } {
#=============================================================
  global cycle_name
  global cycle_init_flag

   set cycle_init_flag TRUE
   set cycle_name DRILL_DEEP
   CYCLE_SET
}


#=============================================================
proc MOM_drill_deep_move { } {
#=============================================================
   global cycle_init_flag


   ABORT_EVENT_CHECK

   PB_CMD_set_cycle_plane
   PB_CMD_force_cycle_parameters
   PB_CMD_remove_Q0

   MOM_do_template cycle_drill_deep

   if { [PB_CMD__check_block_output_cycle_retract] } {
      MOM_do_template cycle_return
   }
   set cycle_init_flag FALSE
}


#=============================================================
proc MOM_drill_dwell { } {
#=============================================================
  global cycle_name
  global cycle_init_flag

   set cycle_init_flag TRUE
   set cycle_name DRILL_DWELL
   CYCLE_SET
}


#=============================================================
proc MOM_drill_dwell_move { } {
#=============================================================
   global cycle_init_flag


   ABORT_EVENT_CHECK

   PB_CMD_set_cycle_plane
   PB_CMD_force_cycle_parameters

   MOM_do_template cycle_drill_dwell

   if { [PB_CMD__check_block_output_cycle_retract] } {
      MOM_do_template cycle_return
   }
   set cycle_init_flag FALSE
}


#=============================================================
proc MOM_drill_text { } {
#=============================================================
  global cycle_name
  global cycle_init_flag

   set cycle_init_flag TRUE
   set cycle_name DRILL_TEXT
   CYCLE_SET
}


#=============================================================
proc MOM_drill_text_move { } {
#=============================================================
   global cycle_init_flag


   ABORT_EVENT_CHECK

   set cycle_init_flag FALSE
}


#=============================================================
proc MOM_end_of_path { } {
#=============================================================
  global mom_sys_add_cutting_time mom_sys_add_non_cutting_time
  global mom_cutting_time mom_machine_time

   if { ![info exists mom_sys_add_cutting_time] } {
      set mom_sys_add_cutting_time 0.0
   }
   if { ![info exists mom_sys_add_non_cutting_time] } {
      set mom_sys_add_non_cutting_time 0.0
   }

  # Accumulated time should be in minutes.
   set mom_cutting_time [expr $mom_cutting_time + $mom_sys_add_cutting_time]
   set mom_machine_time [expr $mom_machine_time + $mom_sys_add_cutting_time + $mom_sys_add_non_cutting_time]
   MOM_reload_variable mom_cutting_time
   MOM_reload_variable mom_machine_time

   if [CMD_EXIST PB_CMD_kin_end_of_path] {
      PB_CMD_kin_end_of_path
   }

   PB_CMD_reset_output_mode

   if { [PB_CMD__check_block_output_coolant_off] } {
      MOM_force Once M_coolant
      MOM_do_template coolant_off
   }
   PB_CMD_unset_parameter
   global mom_sys_in_operation
   set mom_sys_in_operation 0
}


#=============================================================
proc MOM_end_of_subop_path { } {
#=============================================================
}


#=============================================================
proc MOM_first_move { } {
#=============================================================
  global mom_feed_rate mom_feed_rate_per_rev mom_motion_type
  global mom_kin_max_fpm mom_motion_event

   COOLANT_SET ; CUTCOM_SET ; SPINDLE_SET ; RAPID_SET

   PB_CMD_header_operation

   if { [PB_CMD__check_block_output_cancel_dynamic_work_offset_first_move] } {
      MOM_force Once G_dwo
      MOM_do_template cancel_dynamic_work_offset
   }

   if { [PB_CMD__check_block_output_codes_when_tool_axis_change_first_move] } {
      MOM_force Once G_motion G_offset Z
      MOM_do_template return_home_z
   }

   if { [PB_CMD__check_block_output_spindle_rpm_first_move] } {
      MOM_force Once S M_spindle
      MOM_do_template spindle_rpm
   }
   PB_CMD_detect_tool_path_type

   if { [PB_CMD__check_block_output_unclamp_codes_first_move] } {
      MOM_force Once M_clamp_5th Text
      MOM_do_template fifth_axis_unclamp
   }

   if { [PB_CMD__check_block_output_unclamp_codes_first_move] } {
      MOM_force Once M_clamp_4th Text
      MOM_do_template fourth_axis_unclamp
   }

   if { [PB_CMD__check_block_output_initial_move_rotation_first_move] } {
      MOM_force Once G_motion fourth_axis fifth_axis
      MOM_do_template initial_move_rotation
   }

   if { [PB_CMD__check_block_swiveling_coord_rot_first_move] } {
      MOM_do_template activate_dynamic_work_offset
   }

   if { [PB_CMD__check_block_output_clamp_codes_first_move] } {
      MOM_force Once M_clamp_5th Text
      MOM_do_template fifth_axis_clamp
   }

   if { [PB_CMD__check_block_output_clamp_codes_first_move] } {
      MOM_force Once M_clamp_4th Text
      MOM_do_template fourth_axis_clamp
   }
   PB_CMD_recalculate_drilling_parameters_under_auto3d_condition
   PB_CMD_recalculate_initial_pos_with_no_clearance_plane_for_cycle
   PB_CMD_set_tcp_code
   PB_CMD_init_force_address
   PB_CMD_position_tool_to_R_point_with_no_clearance_plane

   if { [PB_CMD__check_block_output_activate_tool_center_point_control] } {
      MOM_force Once G_tcpc H
      MOM_do_template activate_tool_center_point_control
   }
   catch { MOM_$mom_motion_event }

  # Configure turbo output settings
   if { [CMD_EXIST CONFIG_TURBO_OUTPUT] } {
      CONFIG_TURBO_OUTPUT
   }
}


#=============================================================
proc MOM_first_tool { } {
#=============================================================
  global mom_sys_first_tool_handled

  # First tool only gets handled once
   if { [info exists mom_sys_first_tool_handled] } {
      MOM_tool_change
      return
   }

   set mom_sys_first_tool_handled 1

   MOM_tool_change
}


#=============================================================
proc MOM_from_move { } {
#=============================================================
  global mom_feed_rate mom_feed_rate_per_rev  mom_motion_type mom_kin_max_fpm

   COOLANT_SET ; CUTCOM_SET ; SPINDLE_SET ; RAPID_SET

}


#=============================================================
proc MOM_gohome_move { } {
#=============================================================
   MOM_rapid_move
}


#=============================================================
proc MOM_haas_4th_limits { } {
#=============================================================
   global mom_haas_fourth_axis_limits_command_status
   global mom_fourth_axis_limits_group
   global mom_haas_fourth_axis_max_limit
   global mom_haas_fourth_axis_min_limit
   global mom_fourth_axis_limits_group_end
}


#=============================================================
proc MOM_head { } {
#=============================================================
   global mom_head_name
}


#=============================================================
proc MOM_Head { } {
#=============================================================
   MOM_head
}


#=============================================================
proc MOM_HEAD { } {
#=============================================================
   MOM_head
}


#=============================================================
proc MOM_helix_move { } {
#=============================================================
   PB_CMD_helix_move
}


#=============================================================
proc MOM_initial_move { } {
#=============================================================
  global mom_feed_rate mom_feed_rate_per_rev mom_motion_type
  global mom_kin_max_fpm mom_motion_event

   COOLANT_SET ; CUTCOM_SET ; SPINDLE_SET ; RAPID_SET

   PB_CMD_header_operation

   MOM_force Once G_motion G_offset Z
   MOM_do_template return_home_z

   MOM_do_template go_home_xybc

   if { [PB_CMD__check_block_output_return_home_bc_first_tool] } {
      MOM_force Once G_mode G fourth_axis fifth_axis
      MOM_do_template return_home_bc
   }

   if { [PB_CMD__check_block_output_return_home_bc_first_tool] } {
      MOM_force Once G_mode
      MOM_do_template absolute_mode
   }

   MOM_force Once T M
   MOM_do_template tool_select

   MOM_force Once T
   MOM_do_template tool_preselect

   MOM_force Once M
   MOM_do_template opstop

   MOM_force Once G_motion G_offset Z
   MOM_do_template return_home_z

   MOM_force Once S M_spindle
   MOM_do_template spindle_rpm

   MOM_force Once G_plane G_mode G_offset
   MOM_do_template fixture_offset
   PB_CMD_detect_tool_path_type

   if { [PB_CMD__check_block_output_unclamp_codes_initial_move] } {
      MOM_force Once M_clamp_5th Text
      MOM_do_template fifth_axis_unclamp
   }

   if { [PB_CMD__check_block_output_unclamp_codes_initial_move] } {
      MOM_force Once M_clamp_4th Text
      MOM_do_template fourth_axis_unclamp
   }

   if { [PB_CMD__check_block_output_initial_move_rotation_initial_move] } {
      MOM_force Once G_motion fourth_axis fifth_axis
      MOM_do_template initial_move_rotation
   }

   if { [PB_CMD__check_block_swiveling_coord_rot_initial_move] } {
      MOM_force Once G_dwo
      MOM_do_template activate_dynamic_work_offset
   }

   if { [PB_CMD__check_block_output_clamp_codes_initial_move] } {
      MOM_force Once M_clamp_5th Text
      MOM_do_template fifth_axis_clamp
   }

   if { [PB_CMD__check_block_output_clamp_codes_initial_move] } {
      MOM_force Once M_clamp_4th Text
      MOM_do_template fourth_axis_clamp
   }
   PB_CMD_recalculate_drilling_parameters_under_auto3d_condition
   PB_CMD_recalculate_initial_pos_with_no_clearance_plane_for_cycle
   PB_CMD_set_tcp_code
   PB_CMD_init_force_address

   if { [PB_CMD__check_block_output_activate_tool_center_point_control] } {
      MOM_force Once G_tcpc H
      MOM_do_template activate_tool_center_point_control
   }

  global mom_programmed_feed_rate
   if { [EQ_is_equal $mom_programmed_feed_rate 0] } {
      MOM_rapid_move
   } else {
      MOM_linear_move
   }

  # Configure turbo output settings
   if { [CMD_EXIST CONFIG_TURBO_OUTPUT] } {
      CONFIG_TURBO_OUTPUT
   }
}


#=============================================================
proc MOM_insert { } {
#=============================================================
   global mom_Instruction
   PB_CMD_MOM_insert
}


#=============================================================
proc MOM_instance_operation_handler { } {
#=============================================================
   global mom_handle_instanced_operations
}


#=============================================================
proc MOM_length_compensation { } {
#=============================================================
   TOOL_SET MOM_length_compensation

   MOM_do_template tool_length_adjust
}


#=============================================================
proc MOM_linear_move { } {
#=============================================================
   ABORT_EVENT_CHECK

   HANDLE_FIRST_LINEAR_MOVE

   PB_CMD_cutcom_setting
   PB_CMD_define_coolant_mode

   if { [PB_CMD__check_block_mill_default_condition] } {
      MOM_do_template linear_move
   }

   if { [PB_CMD__check_block_G435_output] } {
      MOM_force Once tool_axis_I tool_axis_J tool_axis_K
      MOM_do_template linear_move_for_G435
   }
}


#=============================================================
proc MOM_load_tool { } {
#=============================================================
   global mom_tool_change_type mom_manual_tool_change
   global mom_tool_number mom_next_tool_number
   global mom_sys_tool_number_max mom_sys_tool_number_min

   if { $mom_tool_number < $mom_sys_tool_number_min || \
        $mom_tool_number > $mom_sys_tool_number_max } {

      global mom_warning_info
      set mom_warning_info "Tool number to be output ($mom_tool_number) exceeds limits of\
                            ($mom_sys_tool_number_min/$mom_sys_tool_number_max)"
      MOM_catch_warning
   }
}


#=============================================================
proc MOM_lock_axis { } {
#=============================================================
   global mom_lock_axis
   global mom_lock_axis_plane
   global mom_lock_axis_value
   PB_CMD_MOM_lock_axis
}


#=============================================================
proc MOM_nurbs_move { } {
#=============================================================
   PB_CMD_nurbs_move
}


#=============================================================
proc MOM_operator_message { } {
#=============================================================
   global mom_operator_message
   PB_CMD_MOM_operator_message
}


#=============================================================
proc MOM_opskip_off { } {
#=============================================================
   global mom_opskip_text
   PB_CMD_MOM_opskip_off
}


#=============================================================
proc MOM_opskip_on { } {
#=============================================================
   global mom_opskip_text
   PB_CMD_MOM_opskip_on
}


#=============================================================
proc MOM_opstop { } {
#=============================================================
   MOM_do_template opstop
}


#=============================================================
proc MOM_origin { } {
#=============================================================
   global mom_X
   global mom_Y
   global mom_Z
   global mom_origin_text
}


#=============================================================
proc MOM_pprint { } {
#=============================================================
   global mom_pprint
   PB_CMD_MOM_pprint
}


#=============================================================
proc MOM_prefun { } {
#=============================================================
   MOM_do_template prefun
}


#=============================================================
proc MOM_rapid_move { } {
#=============================================================
  global rapid_spindle_inhibit rapid_traverse_inhibit
  global spindle_first is_from
  global mom_cycle_spindle_axis traverse_axis1 traverse_axis2
  global mom_motion_event

   ABORT_EVENT_CHECK

   set spindle_first NONE

   RAPID_SET
   PB_CMD_check_plane_change_for_swiveling
   PB_CMD_output_first_rapid_move

   if { [PB_CMD__check_block_mill_default_condition] } {
      MOM_do_template rapid_traverse
   }

   if { [PB_CMD__check_block_G435_output] } {
      MOM_force Once tool_axis_I tool_axis_J tool_axis_K
      MOM_do_template rapid_move
   }
   PB_CMD_save_last_z_for_drilling_operations
}


#=============================================================
proc MOM_rotate { } {
#=============================================================
   global mom_rotate_axis_type
   global mom_rotation_mode
   global mom_rotation_direction
   global mom_rotation_angle
   global mom_rotation_reference_mode
   global mom_rotation_text
   PB_CMD_MOM_rotate
}


#=============================================================
proc MOM_select_head { } {
#=============================================================
   global mom_head_type
   global mom_head_text
}


#=============================================================
proc MOM_sequence_number { } {
#=============================================================
   global mom_sequence_mode
   global mom_sequence_number
   global mom_sequence_increment
   global mom_sequence_frequency
   global mom_sequence_text
   SEQNO_SET
}


#=============================================================
proc MOM_set_axis { } {
#=============================================================
   global mom_axis_position
   global mom_axis_position_value
}


#=============================================================
proc MOM_set_modes { } {
#=============================================================
   MODES_SET
}


#=============================================================
proc MOM_set_polar { } {
#=============================================================
   global mom_coordinate_output_mode
}


#=============================================================
proc MOM_spindle_rpm { } {
#=============================================================
   SPINDLE_SET

   MOM_force Once S M_spindle
   MOM_do_template spindle_rpm
}


#=============================================================
proc MOM_spindle_off { } {
#=============================================================
   MOM_do_template spindle_off
}


#=============================================================
proc MOM_start_of_path { } {
#=============================================================
  global mom_sys_in_operation
   set mom_sys_in_operation 1

  global first_linear_move ; set first_linear_move 0
   TOOL_SET MOM_start_of_path

  global mom_sys_add_cutting_time mom_sys_add_non_cutting_time
  global mom_sys_machine_time mom_machine_time

   set mom_sys_add_cutting_time 0.0
   set mom_sys_add_non_cutting_time 0.0
   set mom_sys_machine_time $mom_machine_time

   if [CMD_EXIST PB_CMD_kin_start_of_path] {
      PB_CMD_kin_start_of_path
   }

   PB_CMD_init_helix_custom
   PB_CMD_reset_auto_detected_parameter
   PB_CMD_uplevel_HAAS_fourth_axis_limits
}


#=============================================================
proc MOM_start_of_subop_path { } {
#=============================================================
}


#=============================================================
proc MOM_stop { } {
#=============================================================
   MOM_do_template stop
}


#=============================================================
proc MOM_tap { } {
#=============================================================
  global cycle_name
  global cycle_init_flag

   set cycle_init_flag TRUE
   set cycle_name TAP
   CYCLE_SET
}


#=============================================================
proc MOM_tap_move { } {
#=============================================================
   global cycle_init_flag


   ABORT_EVENT_CHECK

   PB_CMD_set_cycle_plane
   PB_CMD_force_cycle_parameters
   PB_CMD_cal_feedrate_by_pitch_and_ss
   PB_CMD_tapping_g_code_string_determine

   MOM_do_template cycle_tap

   if { [PB_CMD__check_block_output_cycle_retract] } {
      MOM_do_template cycle_tap_return
   }
   set cycle_init_flag FALSE
}


#=============================================================
proc MOM_tap_break_chip { } {
#=============================================================
  global cycle_name
  global cycle_init_flag

   set cycle_init_flag TRUE
   set cycle_name TAP_BREAK_CHIP
   CYCLE_SET
}


#=============================================================
proc MOM_tap_break_chip_move { } {
#=============================================================
   global cycle_init_flag


   ABORT_EVENT_CHECK

   PB_CMD_set_cycle_plane
   PB_CMD_force_cycle_parameters
   PB_CMD_cal_feedrate_by_pitch_and_ss
   PB_CMD_tapping_g_code_string_determine_for_rigid_tap
   PB_CMD_output_M29_to_active_rigid_tap

   MOM_do_template cycle_tap_break_chip

   if { [PB_CMD__check_block_output_cycle_retract] } {
      MOM_do_template cycle_tap_break_chip_return
   }
   set cycle_init_flag FALSE
}


#=============================================================
proc MOM_tap_deep { } {
#=============================================================
  global cycle_name
  global cycle_init_flag

   set cycle_init_flag TRUE
   set cycle_name TAP_DEEP
   CYCLE_SET
}


#=============================================================
proc MOM_tap_deep_move { } {
#=============================================================
   global cycle_init_flag


   ABORT_EVENT_CHECK

   PB_CMD_set_cycle_plane
   PB_CMD_force_cycle_parameters
   PB_CMD_cal_feedrate_by_pitch_and_ss
   PB_CMD_tapping_g_code_string_determine_for_rigid_tap
   PB_CMD_output_M29_to_active_rigid_tap

   MOM_do_template cycle_tap_deep

   if { [PB_CMD__check_block_output_cycle_retract] } {
      MOM_do_template cycle_tap_deep_return
   }
   set cycle_init_flag FALSE
}


#=============================================================
proc MOM_tap_float { } {
#=============================================================
  global cycle_name
  global cycle_init_flag

   set cycle_init_flag TRUE
   set cycle_name TAP_FLOAT
   CYCLE_SET
}


#=============================================================
proc MOM_tap_float_move { } {
#=============================================================
   global cycle_init_flag


   ABORT_EVENT_CHECK

   PB_CMD_set_cycle_plane
   PB_CMD_force_cycle_parameters
   PB_CMD_cal_feedrate_by_pitch_and_ss
   PB_CMD_tapping_g_code_string_determine_for_float_tap

   MOM_do_template cycle_tap_float

   if { [PB_CMD__check_block_output_cycle_retract] } {
      MOM_do_template cycle_tap_float_return
   }
   set cycle_init_flag FALSE
}


#=============================================================
proc MOM_text { } {
#=============================================================
   global mom_user_defined_text
   PB_CMD_MOM_text
}


#=============================================================
proc MOM_tool_change { } {
#=============================================================
   global mom_tool_change_type mom_manual_tool_change
   global mom_tool_number mom_next_tool_number
   global mom_sys_tool_number_max mom_sys_tool_number_min

   if { $mom_tool_number < $mom_sys_tool_number_min || \
        $mom_tool_number > $mom_sys_tool_number_max } {

      global mom_warning_info
      set mom_warning_info "Tool number to be output ($mom_tool_number) exceeds limits of\
                            ($mom_sys_tool_number_min/$mom_sys_tool_number_max)"
      MOM_catch_warning
   }

   if { [info exists mom_tool_change_type] } {
      switch $mom_tool_change_type {
         MANUAL { PB_manual_tool_change }
         AUTO   { PB_auto_tool_change }
      }
   } elseif { [info exists mom_manual_tool_change] } {
      if { ![string compare $mom_manual_tool_change "TRUE"] } {
         PB_manual_tool_change
      }
   }
}


#=============================================================
proc MOM_tool_path_type { } {
#=============================================================
   global mom_ude_5axis_tool_path
}


#=============================================================
proc MOM_tool_preselect { } {
#=============================================================
   global mom_tool_preselect_number mom_tool_number mom_next_tool_number
   global mom_sys_tool_number_max mom_sys_tool_number_min

   if { [info exists mom_tool_preselect_number] } {
      if { $mom_tool_preselect_number < $mom_sys_tool_number_min || \
           $mom_tool_preselect_number > $mom_sys_tool_number_max } {

         global mom_warning_info
         set mom_warning_info "Preselected Tool number ($mom_tool_preselect_number) exceeds limits of\
                               ($mom_sys_tool_number_min/$mom_sys_tool_number_max)"
         MOM_catch_warning
      }

      set mom_next_tool_number $mom_tool_preselect_number
   }

   MOM_do_template tool_preselect
}


#=============================================================
proc MOM_zero { } {
#=============================================================
   global mom_work_coordinate_number
}


#=============================================================
proc PB_approach_move { } {
#=============================================================
}


#=============================================================
proc PB_auto_tool_change { } {
#=============================================================
   global mom_tool_number mom_next_tool_number
   if { ![info exists mom_next_tool_number] } {
      set mom_next_tool_number $mom_tool_number
   }

   PB_CMD_tool_data
}


#=============================================================
proc PB_engage_move { } {
#=============================================================
}


#=============================================================
proc PB_first_cut { } {
#=============================================================
}


#=============================================================
proc PB_first_linear_move { } {
#=============================================================
  global mom_sys_first_linear_move

  # Set this variable to signal 1st linear move has been handled.
   set mom_sys_first_linear_move 1

}


#=============================================================
proc PB_manual_tool_change { } {
#=============================================================
   MOM_do_template stop
}


#=============================================================
proc PB_retract_move { } {
#=============================================================
}


#=============================================================
proc PB_return_move { } {
#=============================================================
}


#=============================================================
proc PB_start_of_program { } {
#=============================================================
   if [CMD_EXIST PB_CMD_kin_start_of_program] {
      PB_CMD_kin_start_of_program
   }

   PB_CMD_customize_output_mode
   PB_CMD_spindle_orient
   PB_CMD_fix_RAPID_SET
   PB_CMD_uplevel_ROTARY_AXIS_RETRACT
   MOM_set_seq_off

   MOM_force Once Text
   MOM_do_template rewind_stop_code
   PB_CMD_program_header
   PB_CMD_tool_list
   PB_CMD_initial_prog_variables
   MOM_set_seq_on

   MOM_force Once G_cutcom G_plane G_feed G_return G_mode G_motion G_adjust G
   MOM_do_template initial_mode_setting_for_program

   if [CMD_EXIST PB_CMD_kin_start_of_program_2] {
      PB_CMD_kin_start_of_program_2
   }
}


#=============================================================
proc PB_user_defined_axis_limit_action { } {
#=============================================================
}


#=============================================================
proc PB_user_def_axis_limit_action { args } {
#=============================================================
}


#=============================================================
proc USER_DEF_AXIS_LIMIT_ACTION { args } {
#=============================================================
}


#=============================================================
proc PB_CMD_FEEDRATE_NUMBER { } {
#=============================================================
#  This custom command is called by FEEDRATE_SET;
#  it allows you to modify the feed rate number after being
#  calculated by the system.
#
#<03-13-08 gsl> - Added use of frn factor (defined in ugpost_base.tcl) & max frn here
#                 Use global frn factor (defined as 1.0 in ugpost_base.tcl) or
#                 define a custom one here

global mom_kin_max_frn
global mom_sys_frn_factor
global mom_feed_rate_number

#set mom_sys_frn_factor 1.0

set f 0.0

if { [info exists mom_feed_rate_number] } {
   set f [expr $mom_feed_rate_number * $mom_sys_frn_factor]

   if { [EQ_is_gt $f $mom_kin_max_frn] } {
      set f $mom_kin_max_frn
      }
   }

return $f
}


#=============================================================
proc PB_CMD_FEEDRATE_SET { } {
#=============================================================
# This custom command will be executed automatically in
# MOM_before_motion event handler.
# Important! Don't change following sentence unless you know what are you doing.

global feed_mode
global mom_machine_mode
global mom_feed_rate_mode

if { $mom_machine_mode == "TURN" } {
   set feed_mode $mom_feed_rate_mode
   }
}


#=============================================================
proc PB_CMD_MOM_clamp { } {
#=============================================================
# Default handler for UDE MOM_clamp
# - Do not attach it to any event!

global mom_clamp_axis
global mom_clamp_status
global mom_sys_auto_clamp

if { ![string compare "AUTO" $mom_clamp_axis] } {
   if { ![string compare "ON" $mom_clamp_status] } {
      set mom_sys_auto_clamp "ON"

      } elseif { ![string compare "OFF" $mom_clamp_status] } {
               set mom_sys_auto_clamp "OFF"
               }

   } else {
          CATCH_WARNING "$mom_clamp_axis not handled in current implementation!"
          }
}


#=============================================================
proc PB_CMD_MOM_insert { } {
#=============================================================
# Default handler for UDE MOM_insert
# - Do not attach it to any event!
#
# This procedure is executed when the Insert command is activated.

global mom_Instruction

MOM_output_literal "$mom_Instruction"
}


#=============================================================
proc PB_CMD_MOM_lock_axis { } {
#=============================================================
# Default handler for UDE MOM_lock_axis
# - Do not attach it to any event!
#
# 18-Sep-2015 ljt - reset positive_radius, fix PR6961328

global mom_sys_lock_axis
global mom_sys_lock_value
global mom_sys_lock_plane
global mom_sys_lock_status

set status [SET_LOCK axis plane value]

if { ![string compare "error" $status] } {
   global mom_warning_info

   CATCH_WARNING $mom_warning_info

   set mom_sys_lock_status OFF

   } else {
          set mom_sys_lock_status $status

          if { [string compare "OFF" $status] } {
             set mom_sys_lock_axis $axis
             set mom_sys_lock_plane $plane
             set mom_sys_lock_value $value

             LOCK_AXIS_INITIALIZE

             } else {
                    global positive_radius

                    set positive_radius "0"
                    }
          }
}


#=============================================================
proc PB_CMD_MOM_operator_message { } {
#=============================================================
# Default handler for UDE MOM_operator_message
# - Do not attach it to any event!
#
# This procedure is executed when the Operator Message command is activated.
# 08-21-15 szl - Fix PR7471332:Parse error during machine code simulation if the UDE Operator Message is added.

global ptp_file_name
global mom_group_name
global group_output_file
global mom_sys_ptp_output
global mom_sys_control_in
global mom_sys_control_out
global mom_operator_message
global mom_post_in_simulation
global mom_sys_commentary_output
global mom_operator_message_status
global mom_operator_message_defined

if { [info exists mom_operator_message_defined] } {
   if { $mom_operator_message_defined == 0 } {
      return
      }
   }

if { [string compare "ON" $mom_operator_message] && [string compare "OFF" $mom_operator_message] } {
   set brac_start [string first \( $mom_operator_message]
   set brac_end   [string last \) $mom_operator_message]

   if { $brac_start != 0 } {
      set text_string "("

      } else {
             set text_string ""
             }

   append text_string $mom_operator_message

   if { $brac_end != [expr [string length $mom_operator_message] - 1] } {
      append text_string ")"
      }

   MOM_close_output_file $ptp_file_name

   if { [info exists mom_group_name] } {
      if { [info exists group_output_file($mom_group_name)] } {
         MOM_close_output_file $group_output_file($mom_group_name)
         }
      }

   MOM_suppress once N

   if { ![info exists mom_post_in_simulation] || $mom_post_in_simulation == 0 } {
      MOM_output_literal $text_string
      }

   if { ![string compare "ON" $mom_sys_ptp_output] } {
      MOM_open_output_file $ptp_file_name
      }

   if { [info exists mom_group_name] } {
      if { [info exists group_output_file($mom_group_name)] } {
         MOM_open_output_file $group_output_file($mom_group_name)
         }
      }

   set need_commentary $mom_sys_commentary_output
   set mom_sys_commentary_output OFF

   regsub -all {[)]} $text_string $mom_sys_control_in text_string
   regsub -all {[(]} $text_string $mom_sys_control_out text_string

   MOM_output_literal $text_string

   set mom_sys_commentary_output $need_commentary

   } else {
          set mom_operator_message_status $mom_operator_message
          }
}


#=============================================================
proc PB_CMD_MOM_opskip_off { } {
#=============================================================
# Default handler for UDE MOM_opskip_off
# - Do not attach it to any event!
#
# This procedure is executed when the Optional skip command is activated.

global mom_sys_opskip_block_leader

MOM_set_line_leader off $mom_sys_opskip_block_leader
}


#=============================================================
proc PB_CMD_MOM_opskip_on { } {
#=============================================================
# Default handler for UDE MOM_opskip_on
# - Do not attach it to any event!
#
# This procedure is executed when the Optional skip command is activated.

global mom_sys_opskip_block_leader

MOM_set_line_leader always $mom_sys_opskip_block_leader
}


#=============================================================
proc PB_CMD_MOM_pprint { } {
#=============================================================
# Default handler for UDE MOM_pprint
# - Do not attach it to any event!
#
# This procedure is executed when the PPrint command is activated.

global mom_pprint_defined

if { [info exists mom_pprint_defined] } {
   if { $mom_pprint_defined == 0 } {
      return
      }
   }

PPRINT_OUTPUT
}


#=============================================================
proc PB_CMD_MOM_rotate { } {
#=============================================================
# Default handler for UDE MOM_rotate
# - Do not attach it to any event!

## <rws 04-11-2008>
## If in TURN mode and user invokes "Flip tool aorund Holder" a MOM_rotate event is generated
## When this happens ABORT this event via return

global mom_machine_mode


if { [info exists mom_machine_mode] && [string match "TURN" $mom_machine_mode] } {
   if [CMD_EXIST PB_CMD_handle_lathe_flash_tool] {
      PB_CMD_handle_lathe_flash_tool
      }

   return
   }

global mom_pos
global mom_prev_pos
global mom_sys_leader
global mom_rotation_mode
global mom_out_angle_pos
global unlocked_prev_pos
global mom_rotation_angle
global mom_rotate_axis_type
global mom_prev_rot_ang_4th
global mom_prev_rot_ang_5th
global mom_kin_machine_type
global mom_rotation_direction
global mom_kin_4th_axis_leader
global mom_kin_5th_axis_leader
global mom_kin_4th_axis_min_limit
global mom_kin_4th_axis_max_limit
global mom_kin_5th_axis_min_limit
global mom_kin_5th_axis_max_limit
global mom_kin_4th_axis_direction
global mom_kin_5th_axis_direction
global mom_rotation_reference_mode

if { ![info exists mom_rotation_angle] } {
   # Should the event be aborted here???
   return
   }

if { ![info exists mom_kin_5th_axis_direction] } {
   set mom_kin_5th_axis_direction "0"
   }

#Determine which rotary axis the UDE has specifid - fourth(3), fifth(4) or invalid(0)
if { [string match "*3_axis_mill_turn*" $mom_kin_machine_type] } {
   switch $mom_rotate_axis_type {
                            CAXIS -
                      FOURTH_AXIS -
                            TABLE {
                                  set axis 3
                                  }
                          default {
                                  set axis 0
                                  }
                                }

   } else {
          switch $mom_rotate_axis_type {
                                   AAXIS -
                                   BAXIS -
                                   CAXIS {
                                         set axis [AXIS_SET $mom_rotate_axis_type]
                                         }
                                    HEAD {
                                         if { ![string compare "5_axis_head_table" $mom_kin_machine_type] || ![string compare "5_AXIS_HEAD_TABLE" $mom_kin_machine_type] } {
                                            set axis 4

                                            } else {
                                                   set axis 3
                                                   }
                                         }
                              FIFTH_AXIS {
                                         set axis 4
                                         }
                             FOURTH_AXIS -
                                   TABLE -
                                 default {
                                         set axis 3
                                         }
                                       }
          }

if { $axis == 0 } {
   CATCH_WARNING "Invalid rotary axis"

   MOM_abort_event
   }

switch $mom_rotation_mode {
                       NONE -
                    ATANGLE {
                            set angle $mom_rotation_angle
                            set mode 0
                            }
                   ABSOLUTE {
                            set angle $mom_rotation_angle
                            set mode 1
                            }
                INCREMENTAL {
                            set angle [expr $mom_pos($axis) + $mom_rotation_angle]
                            set mode 0
                            }
                          }

switch $mom_rotation_direction {
                            NONE {
                                 set dir 0
                                 }
                             CLW {
                                 set dir 1
                                 }
                            CCLW {
                                 set dir -1
                                 }
                               }

set ang [LIMIT_ANGLE $angle]
set mom_pos($axis) $ang

if { $axis == "3" } { ;# Rotate 4th axis
   if { ![info exists mom_prev_rot_ang_4th] } {
      set mom_prev_rot_ang_4th [MOM_ask_address_value fourth_axis]
      }

   if { [string length [string trim $mom_prev_rot_ang_4th]] == 0 } {
      set mom_prev_rot_ang_4th 0.0
      }

   set prev_angles(0) $mom_prev_rot_ang_4th

   } elseif { $axis == "4" } { ;# Rotate 5th axis
            if { ![info exists mom_prev_rot_ang_5th] } {
               set mom_prev_rot_ang_5th [MOM_ask_address_value fifth_axis]
               }

            if { [string length [string trim $mom_prev_rot_ang_5th]] == 0 } {
               set mom_prev_rot_ang_5th 0.0
               }

            set prev_angles(1) $mom_prev_rot_ang_5th
            }

set p [expr $axis + 1]
set a [expr $axis - 3]

if { $axis == 3  &&  [string match "MAGNITUDE_DETERMINES_DIRECTION" $mom_kin_4th_axis_direction] } {
   set dirtype "MAGNITUDE_DETERMINES_DIRECTION"

   global mom_sys_4th_axis_dir_mode

   if { [info exists mom_sys_4th_axis_dir_mode] && ![string compare "ON" $mom_sys_4th_axis_dir_mode] } {
      set del $dir

      if { $del == 0 } {
         set del [expr $ang - $mom_prev_pos(3)]

         if { $del >  180.0 } { set del [expr $del - 360.0] }
         if { $del < -180.0 } { set del [expr $del + 360.0] }
         }

      global mom_sys_4th_axis_cur_dir
      global mom_sys_4th_axis_clw_code
      global mom_sys_4th_axis_cclw_code

      if { $del > 0.0 } {
         set mom_sys_4th_axis_cur_dir $mom_sys_4th_axis_clw_code

         } elseif { $del < 0.0 } {
                  set mom_sys_4th_axis_cur_dir $mom_sys_4th_axis_cclw_code
                  }
      }

   } elseif { $axis == 4  &&  [string match "MAGNITUDE_DETERMINES_DIRECTION" $mom_kin_5th_axis_direction] } {
            set dirtype "MAGNITUDE_DETERMINES_DIRECTION"

            global mom_sys_5th_axis_dir_mode

            if { [info exists mom_sys_5th_axis_dir_mode] && ![string compare "ON" $mom_sys_5th_axis_dir_mode] } {
               set del $dir

               if { $del == 0 } {
                  set del [expr $ang - $mom_prev_pos(4)]

                  if { $del >  180.0 } { set del [expr $del - 360.0] }
                  if { $del < -180.0 } { set del [expr $del + 360.0] }
                  }

               global mom_sys_5th_axis_cur_dir
               global mom_sys_5th_axis_clw_code
               global mom_sys_5th_axis_cclw_code

               if { $del > 0.0 } {
                  set mom_sys_5th_axis_cur_dir $mom_sys_5th_axis_clw_code

                  } elseif { $del < 0.0 } {
                           set mom_sys_5th_axis_cur_dir $mom_sys_5th_axis_cclw_code
                           }
               }

            } else {
                   set dirtype "SIGN_DETERMINES_DIRECTION"
                   }

if { $mode == 1 } {
   set mom_out_angle_pos($a) $angle

   } elseif { [string match "MAGNITUDE_DETERMINES_DIRECTION" $dirtype] } {
            if { $axis == 3 } {
               set mom_out_angle_pos($a) [ROTSET $ang $prev_angles(0) $mom_kin_4th_axis_direction $mom_kin_4th_axis_leader mom_sys_leader(fourth_axis) $mom_kin_4th_axis_min_limit $mom_kin_4th_axis_max_limit]

               } else {
                      set mom_out_angle_pos($a) [ROTSET $ang $prev_angles(1) $mom_kin_5th_axis_direction $mom_kin_5th_axis_leader mom_sys_leader(fifth_axis) $mom_kin_5th_axis_min_limit $mom_kin_5th_axis_max_limit]
                      }

            } elseif { [string match "SIGN_DETERMINES_DIRECTION" $dirtype] } {
                     if { $dir == -1 } {
                        if { $axis == 3 } {
                           set mom_sys_leader(fourth_axis) $mom_kin_4th_axis_leader-

                           } else {
                                  set mom_sys_leader(fifth_axis) $mom_kin_5th_axis_leader-
                                  }

                        } elseif { $dir == 0 } {
                                 if { $axis == 3 } {
                                    set mom_out_angle_pos($a) [ROTSET $ang $prev_angles(0) $mom_kin_4th_axis_direction $mom_kin_4th_axis_leader mom_sys_leader(fourth_axis) $mom_kin_4th_axis_min_limit $mom_kin_4th_axis_max_limit]

                                    } else {
                                           set mom_out_angle_pos($a) [ROTSET $ang $prev_angles(1) $mom_kin_5th_axis_direction $mom_kin_5th_axis_leader mom_sys_leader(fifth_axis) $mom_kin_5th_axis_min_limit $mom_kin_5th_axis_max_limit]
                                           }

                                 } elseif { $dir == 1 } {
                                          set mom_out_angle_pos($a) $ang
                                          }
                     }

global mom_sys_auto_clamp

if { [info exists mom_sys_auto_clamp] && [string match "ON" $mom_sys_auto_clamp] } {
   set out1 "1"
   set out2 "0"

   if { $axis == 3 } { ; #Rotate 4th axis
      AUTO_CLAMP_2 $out1
      AUTO_CLAMP_1 $out2

      } else {
             AUTO_CLAMP_1 $out1
             AUTO_CLAMP_2 $out2
             }
   }

if { $axis == 3 } {
   ####  <rws>
   ####  Use ROTREF switch ON to not output the actual 4th axis move
   if { ![string compare "OFF" $mom_rotation_reference_mode] } {
      PB_CMD_fourth_axis_rotate_move
      }

   if { ![string compare "SIGN_DETERMINES_DIRECTION" $mom_kin_4th_axis_direction] } {
      set mom_prev_rot_ang_4th [expr abs($mom_out_angle_pos(0))]

      } else {
             set mom_prev_rot_ang_4th $mom_out_angle_pos(0)
             }

   MOM_reload_variable mom_prev_rot_ang_4th

   } else {
          if { [info exists mom_kin_5th_axis_direction] } {
             ####  <rws>
             ####  Use ROTREF switch ON to not output the actual 5th axis move
             if { ![string compare "OFF" $mom_rotation_reference_mode] } {
                #PB_CMD_fifth_axis_rotate_move
                }

             if { ![string compare "SIGN_DETERMINES_DIRECTION" $mom_kin_5th_axis_direction] } {
                set mom_prev_rot_ang_5th [expr abs($mom_out_angle_pos(1))]

                } else {
                       set mom_prev_rot_ang_5th $mom_out_angle_pos(1)
                       }

             MOM_reload_variable mom_prev_rot_ang_5th
             }
          }

#<05-10-06 sws> pb351 - Uncommented next 3 lines
#<01-07-10 wbh> Reset mom_prev_pos using the variable mom_out_angle_pos
# set mom_prev_pos($axis) $ang
set mom_prev_pos($axis) $mom_out_angle_pos([expr $axis-3])

MOM_reload_variable -a mom_prev_pos
MOM_reload_variable -a mom_out_angle_pos
}


#=============================================================
proc PB_CMD_MOM_text { } {
#=============================================================
# Default handler for UDE MOM_text
# - Do not attach it to any event!
#
# This procedure is executed when the Text command is activated.

global mom_record_fields
global mom_pprint_defined
global mom_sys_control_in
global mom_sys_control_out
global mom_user_defined_text
global mom_operator_message_defined
global mom_record_text mom_pprint set mom_Instruction mom_operator_message

switch $mom_record_fields(0) {
                       "PPRINT"
                              {
                              set mom_pprint_defined 1
                              set mom_pprint $mom_record_text

                              MOM_pprint
                              }
                       "INSERT"
                              {
                              set mom_Instruction $mom_record_text

                              MOM_insert
                              }
                       "DISPLY"
                              {
                              set mom_operator_message_defined 1
                              set mom_operator_message $mom_record_text

                              MOM_operator_message
                              }
                        default
                              {
                              if { [info exists mom_user_defined_text] } {
                                 MOM_output_literal "${mom_sys_control_out}${mom_user_defined_text}${mom_sys_control_in}"
                                 }
                              }
                             }
}


#=============================================================
proc PB_CMD__catch_warning { } {
#=============================================================
# This command will be called by PB_catch_warning when warning
# conditions arise while running a multi-axis post.
#
# - Warning message "mom_warning_info" can be transfered to
#   "mom_sys_rotary_error" to cause ROTARY_AXIS_RETRACT to be
#   executed in MOM_before_motion, which allows the post to
#   interrupt the normal output of a multi-axis linear move.
#   Depending on the option set for handling the rotary axis'
#   limit violation, the rotary angles may be recomputed.
#
# - Certain warning situations require post to abort subsequent event or
#   the entire posting job. This can be carried out by setting the variable
#   "mom_sys_abort_next_event" to different severity levels.
#   PB_CMD_abort_event can be customized to handle the conditions accordingly.
#
#
# REVISIONS
# Oct-26-2017 gsl - Added new condition of "*ROTARY OUT OF LIMIT - Previous rotary position used*"
#

  #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  # Define ::mom_sys_rotary_error to execute ROTARY_AXIS_RETRACT
  #
   set rotary_limit_error 0

   if { [string match "*Previous rotary axis solution degenerated*" $::mom_warning_info] } {
     # Default to "0" to avoid regression
      set rotary_limit_error 0
   }


   if { [string match "*ROTARY OUT OF LIMIT - Previous rotary position used*" $::mom_warning_info] } {
     ## Set next variable to cause current motion to be aborted.
     # - Comment out next line to ignore this condition -
      set ::mom_sys_abort_next_event 1

     ## Uncomment next line to handle this condition with retract/re-engage.
     # set ::mom_warning_info "ROTARY CROSSING LIMIT."
   }


   if { [string match "ROTARY CROSSING LIMIT." $::mom_warning_info] } {
      set rotary_limit_error 1
   }


   if { [string match "secondary rotary position being used" $::mom_warning_info] } {
      set rotary_limit_error 1
   }


   if { $rotary_limit_error } {
      UNSET_VARS ::mom_sys_abort_next_event
      set ::mom_sys_rotary_error $::mom_warning_info
return
   }


  #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  # Define ::mom_sys_abort_next_event to abort subsequent event
  #
   if { [string match "WARNING: unable to determine valid rotary positions" $::mom_warning_info] } {

     # To abort next event (in PB_CMD_abort_event)
     #
      set ::mom_sys_abort_next_event 1

     # - Whoever handles the condition MUST unset "::mom_sys_abort_next_event"!
   }
}


#=============================================================
proc PB_CMD__check_block_G435_output { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output
#Deal with the condition about G43.5 that using vector to output

global mom_machine_mode
global mom_sys_adjust_code

global dpp_ge

if { ![string compare $mom_machine_mode "MILL"] } {
   if { $dpp_ge(sys_tcp_tool_axis_output_mode) == "VECTOR" && $dpp_ge(toolpath_axis_num) == "5" } {
      return 1

      } else {
             return 0
             }

   } else {
          return 0
          }
}


#=============================================================
proc PB_CMD__check_block_G52 { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

   global mom_logname
   global mom_special_output
   global mom_mcs_info
   global mom_mcsname_attach_opr
   global mom_operation_name
   global mom_g52_origin
   global mom_parent_csys_matrix
   global G52_active

   MOM_ask_mcs_info

   set G52_active "inactive"

   if { [info exists mom_special_output] && $mom_special_output == 0 } {
      set mcs_name $mom_mcsname_attach_opr($mom_operation_name)
      set parent_name $mom_mcs_info($mcs_name,parent)
      if { [string compare $parent_name ""] && $mom_mcs_info($parent_name,purpose) == 0 } {
         if { $mom_mcs_info($parent_name,output_type) == 2 } {
            #set mom_g52_origin(0) $mom_parent_csys_matrix(9)
            #set mom_g52_origin(1) $mom_parent_csys_matrix(10)
            #set mom_g52_origin(2) $mom_parent_csys_matrix(11)

            set g52_name $mcs_name
            set fixture $parent_name
            set point(0) [expr $mom_mcs_info($g52_name,org,0)-$mom_mcs_info($fixture,org,0)]
            set point(1) [expr $mom_mcs_info($g52_name,org,1)-$mom_mcs_info($fixture,org,1)]
            set point(2) [expr $mom_mcs_info($g52_name,org,2)-$mom_mcs_info($fixture,org,2)]

            set matrix(0) $mom_mcs_info($fixture,xvec,0)
            set matrix(1) $mom_mcs_info($fixture,xvec,1)
            set matrix(2) $mom_mcs_info($fixture,xvec,2)
            set matrix(3) $mom_mcs_info($fixture,yvec,0)
            set matrix(4) $mom_mcs_info($fixture,yvec,1)
            set matrix(5) $mom_mcs_info($fixture,yvec,2)
            set matrix(6) $mom_mcs_info($fixture,zvec,0)
            set matrix(7) $mom_mcs_info($fixture,zvec,1)
            set matrix(8) $mom_mcs_info($fixture,zvec,2)

            MTX3_vec_multiply point matrix mom_g52_origin
            set G52_active "active"
 return 1
         } elseif { $mom_mcs_info($parent_name,output_type) == 0 } {
            set g52_name $parent_name
            set fixture $mom_mcs_info($g52_name,parent)
            set point(0) [expr $mom_mcs_info($g52_name,org,0)-$mom_mcs_info($fixture,org,0)]
            set point(1) [expr $mom_mcs_info($g52_name,org,1)-$mom_mcs_info($fixture,org,1)]
            set point(2) [expr $mom_mcs_info($g52_name,org,2)-$mom_mcs_info($fixture,org,2)]

            set matrix(0) $mom_mcs_info($fixture,xvec,0)
            set matrix(1) $mom_mcs_info($fixture,xvec,1)
            set matrix(2) $mom_mcs_info($fixture,xvec,2)
            set matrix(3) $mom_mcs_info($fixture,yvec,0)
            set matrix(4) $mom_mcs_info($fixture,yvec,1)
            set matrix(5) $mom_mcs_info($fixture,yvec,2)
            set matrix(6) $mom_mcs_info($fixture,zvec,0)
            set matrix(7) $mom_mcs_info($fixture,zvec,1)
            set matrix(8) $mom_mcs_info($fixture,zvec,2)

            MTX3_vec_multiply point matrix mom_g52_origin
            set G52_active "active"
 return 1
         }
      }
   } else {
      set mcs_name $mom_mcsname_attach_opr($mom_operation_name)
      set parent_name $mom_mcs_info($mcs_name,parent)
      if { [string compare $parent_name ""] } {
         if { $mom_mcs_info($parent_name,purpose) == 0 } {
            if { $mom_mcs_info($parent_name,output_type) == 0 } {
               set g52_name $parent_name
               set g52_parent $mom_mcs_info($g52_name,parent)
               if { $mom_mcs_info($g52_parent,purpose) == 0 && $mom_mcs_info($g52_parent,output_type) == 2 } {
                   set fixture $g52_parent
               } elseif { $mom_mcs_info($g52_parent,purpose) == 0 && $mom_mcs_info($g52_parent,output_type) == 0 } {
                   set fixture $mom_mcs_info($g52_parent,parent)
               } else {
                   set fixture $g52_name
               }
               set point(0) [expr $mom_mcs_info($g52_name,org,0)-$mom_mcs_info($fixture,org,0)]
               set point(1) [expr $mom_mcs_info($g52_name,org,1)-$mom_mcs_info($fixture,org,1)]
               set point(2) [expr $mom_mcs_info($g52_name,org,2)-$mom_mcs_info($fixture,org,2)]

               set matrix(0) $mom_mcs_info($fixture,xvec,0)
               set matrix(1) $mom_mcs_info($fixture,xvec,1)
               set matrix(2) $mom_mcs_info($fixture,xvec,2)
               set matrix(3) $mom_mcs_info($fixture,yvec,0)
               set matrix(4) $mom_mcs_info($fixture,yvec,1)
               set matrix(5) $mom_mcs_info($fixture,yvec,2)
               set matrix(6) $mom_mcs_info($fixture,zvec,0)
               set matrix(7) $mom_mcs_info($fixture,zvec,1)
               set matrix(8) $mom_mcs_info($fixture,zvec,2)

               MTX3_vec_multiply point matrix mom_g52_origin
               set G52_active "active"
 return 1
            }
         }
      }
   }

 return 0
}


#=============================================================
proc PB_CMD__check_block_G52_cancel { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

  global G52_active

   if { [info exists G52_active] && $G52_active == "active" } {
      return 1
   } else {
      return 0
   }
}


#=============================================================
proc PB_CMD__check_block_HPCC_mode_on { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

  global dpp_ge

   if { $dpp_ge(sys_is_HPCC_mode) == "ON" } {
      return 1
   } else {
      return 0
   }
}


#=============================================================
proc PB_CMD__check_block_auto_align_rotary_axis { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

# Check if G53.1 should output.

global dpp_ge

if { [string compare "NONE" $dpp_ge(coord_rot)] } {
   return 1

   } else {
          return 0
          }
}


#=============================================================
proc PB_CMD__check_block_check_retract_setting { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output
# 07-May-15 Jintao - no output if retract_to_pos < rapid_to_pos

  global mom_cycle_retract_to_pos
  global mom_cycle_rapid_to_pos

  # If operation has retraction, output rapid move to the retraction point
   if { [EQ_is_le $mom_cycle_retract_to_pos(2) $mom_cycle_rapid_to_pos(2)] } {
return 0
   } else {
      MOM_force Once tap_string F R dwell cycle_step
return 1
   }
}


#=============================================================
proc PB_CMD__check_block_cycle_no_clearanceplane_rapidmove { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output
# Deal with initial rapid move of cycle operation that has no clearance plane or start point

  global dpp_ge
  global mom_current_motion

   if { $dpp_ge(cycle_clearance_plane) == "FALSE" && $mom_current_motion == "initial_move" } {
return 1
   } else {
return 0
   }
}


#=============================================================
proc PB_CMD__check_block_cycle_plane_change_for_first_G68 { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

# Check if G68 should output.

  global dpp_ge
  global mom_pos
  global save_mom_kin_machine_type

   if { $save_mom_kin_machine_type == "5_axis_dual_table" || $save_mom_kin_machine_type == "4_axis_table" } {
      return 0
   }

   if { $dpp_ge(toolpath_axis_num) == "3" &&\
        $dpp_ge(sys_coord_rotation_output_type) == "WCS_ROTATION" &&\
        $dpp_ge(cycle_plane_change) } {

      if { ![EQ_is_equal $dpp_ge(coord_rot_angle,0) 0] } {
         set dpp_ge(coord_offset2,0) 0
         set dpp_ge(coord_offset2,1) 0
         set dpp_ge(coord_offset2,2) 0
         MOM_do_template three_plus_two_suppress CREATE

         return 1

      } else {
         set dpp_ge(coord_offset2,0) $dpp_ge(coord_offset,0)
         set dpp_ge(coord_offset2,1) $dpp_ge(coord_offset,1)
         set dpp_ge(coord_offset2,2) $dpp_ge(coord_offset,2)

         return 0
      }
   } else {
      return 0
   }
}


#=============================================================
proc PB_CMD__check_block_cycle_plane_change_for_second_G68 { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

# Check if it's need to output G68 twice.

  global dpp_ge
  global save_mom_kin_machine_type

   if { $save_mom_kin_machine_type == "5_axis_dual_table" || $save_mom_kin_machine_type == "4_axis_table"} {
      return 0
   }

   if { $dpp_ge(toolpath_axis_num) == "3" && $dpp_ge(sys_coord_rotation_output_type) == "WCS_ROTATION" && $dpp_ge(cycle_plane_change) } {
      if { ![EQ_is_equal $dpp_ge(coord_rot_angle,1) 0] ||\
           ![EQ_is_equal $dpp_ge(coord_offset2,0) 0] ||\
           ![EQ_is_equal $dpp_ge(coord_offset2,1) 0] ||\
           ![EQ_is_equal $dpp_ge(coord_offset2,2) 0] } {
         return 1
      } else {
         return 0
      }
   } else {
      return 0
   }
}


#=============================================================
proc PB_CMD__check_block_cycle_plane_change_for_swiveling { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

# Check if tool axis changes between hole and if G68.2 should output.
# 10-18-2013 levi - When tool axis change back to straight, recalculate mom_pos using the original kinematics.

  global mom_pos
  global dpp_ge
  global mom_cycle_rapid_to_pos mom_cycle_rapid_to
  global mom_cycle_retract_to_pos mom_cycle_retract_to
  global mom_cycle_feed_to_pos mom_cycle_feed_to
  global mom_out_angle_pos
  global mom_cycle_feed_to_pos
  global mom_mcs_goto
  global mom_prev_pos
  global mom_prev_out_angle_pos
  global mom_kin_machine_type
  global save_mom_kin_machine_type
  global mom_tool_axis
  global mom_result

# set default value for flag variable dpp_ge(cycle_plane_change)
  if { $dpp_ge(sys_coord_rotation_output_type) == "SWIVELING" } {
     set dpp_ge(cycle_plane_change) FALSE
  }

  if { [string match "*3_axis*" $save_mom_kin_machine_type] || [string match "*4_axis*" $save_mom_kin_machine_type] ||\
       $dpp_ge(coord_rot) == "LOCAL" } {
     return 0
  }

  if { [info exists dpp_ge(ncm_work_plane_change_mode)] && \
       ($dpp_ge(ncm_work_plane_change_mode) != "None" && $dpp_ge(ncm_work_plane_change_mode) != "direct_change") } {
return 0
  }

  if { $dpp_ge(toolpath_axis_num) == "3" && $dpp_ge(sys_coord_rotation_output_type) == "SWIVELING" } {
     set dpp_ge(coord_rot) [DPP_GE_COOR_ROT "ZXZ" angle offset pos]
     for { set i 0 } { $i < 3 } { incr i } {
        if { [info exists offset] } {
           set dpp_ge(coord_offset,$i) $offset($i)
        }
        if { [info exists angle] } {
           set dpp_ge(coord_rot_angle,$i) $angle($i)
        }
        if { [info exists pos] } {
           set mom_pos($i) $pos($i)
        }
     }

     MOM_reload_variable -a mom_pos

     if { [string compare "NONE" $dpp_ge(coord_rot)] } {
        if { ![EQ_is_equal $dpp_ge(coord_rot_angle,0) $dpp_ge(prev_coord_rot_angle,0)] ||\
             ![EQ_is_equal $dpp_ge(coord_rot_angle,1) $dpp_ge(prev_coord_rot_angle,1)] ||\
             ![EQ_is_equal $dpp_ge(coord_rot_angle,2) $dpp_ge(prev_coord_rot_angle,2)] } {
           set dpp_ge(prev_coord_rot_angle,0) $dpp_ge(coord_rot_angle,0)
           set dpp_ge(prev_coord_rot_angle,1) $dpp_ge(coord_rot_angle,1)
           set dpp_ge(prev_coord_rot_angle,2) $dpp_ge(coord_rot_angle,2)
           #Cancle tool length compensation before call G68.2
           MOM_output_literal "G49"
           MOM_output_literal "G69"
           MOM_do_template three_plus_two_suppress CREATE

           MOM_force Once G_motion G_adjust tap_string X Y Z H F R dwell cycle_step
           set mom_cycle_spindle_axis 2
           set dpp_ge(cycle_plane_change) TRUE
           # Recalculate the hole parameters for this hole
           VMOV 3 mom_pos mom_cycle_rapid_to_pos
           VMOV 3 mom_pos mom_cycle_feed_to_pos
           VMOV 3 mom_pos mom_cycle_retract_to_pos
           set mom_cycle_rapid_to_pos(2) [expr $mom_pos(2)+$mom_cycle_rapid_to]
           set mom_cycle_retract_to_pos(2) [expr $mom_pos(2)+$mom_cycle_retract_to]
           set mom_cycle_feed_to_pos(2) [expr $mom_pos(2)+$mom_cycle_feed_to]
           return 1
        } else {
           return 0
        }
     } else {
        set dpp_ge(prev_coord_rot_angle,0) 0
        set dpp_ge(prev_coord_rot_angle,1) 0
        set dpp_ge(prev_coord_rot_angle,2) 0

       # If it's not auto3d condition, restore the kinematics and recalculate mom_pos
        DPP_GE_RESTORE_KINEMATICS
        PB_CMD__convert_point

        MOM_output_literal "G49"
        MOM_output_literal "G69"
        MOM_force Once G_motion G_adjust tap_string X Y Z H F R dwell cycle_step fourth_axis fifth_axis

        VMOV 3 mom_pos mom_cycle_rapid_to_pos
        VMOV 3 mom_pos mom_cycle_feed_to_pos
        VMOV 3 mom_pos mom_cycle_retract_to_pos
        set mom_cycle_rapid_to_pos(2) [expr $mom_pos(2)+$mom_cycle_rapid_to]
        set mom_cycle_retract_to_pos(2) [expr $mom_pos(2)+$mom_cycle_retract_to]
        set mom_cycle_feed_to_pos(2) [expr $mom_pos(2)+$mom_cycle_feed_to]

        MOM_reload_variable -a mom_pos
        return 0
     }
  } else {
     return 0
  }
}


#=============================================================
proc PB_CMD__check_block_cycle_plane_change_for_wcs_rotation { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

# Check if tool axis changes between hole and if rotary angle should output.
# 10-18-2013 levi - When tool axis change back to straight, recalculate mom_pos using the original kinematics.

  global dpp_ge
  global mom_pos
  global mom_kin_machine_type
  global mom_cycle_rapid_to_pos mom_cycle_rapid_to
  global mom_cycle_retract_to_pos mom_cycle_retract_to
  global mom_cycle_feed_to_pos mom_cycle_feed_to
  global mom_out_angle_pos
  global mom_cycle_feed_to_pos
  global mom_mcs_goto
  global mom_prev_out_angle_pos
  global mom_prev_pos
  global save_mom_kin_machine_type
  global mom_tool_axis
  global mom_result

# set default value for flag variable dpp_ge(cycle_plane_change)
  if { $dpp_ge(sys_coord_rotation_output_type) == "WCS_ROTATION" } {
     set dpp_ge(cycle_plane_change) FALSE
  }

  if { [string match "*3_axis*" $save_mom_kin_machine_type] || [string match "*4_axis*" $save_mom_kin_machine_type] ||\
       $dpp_ge(coord_rot) == "LOCAL" || $dpp_ge(sys_coord_rotation_output_type) == "SWIVELING" } {
     return 0
  }

  if { $dpp_ge(toolpath_axis_num) == "3" && $dpp_ge(sys_coord_rotation_output_type) == "WCS_ROTATION" } {
     if { $save_mom_kin_machine_type == "5_axis_dual_table" || $save_mom_kin_machine_type == "4_axis_table" } {
        return 0
     }

     # To avoid the can't save bug of pb, initialize the local variable. Just for sim05 vnc!
     for { set i 0 } { $i < 3 } { incr i } {
        set g68_first_vec($i) 0
        set g68_second_vec($i) 0
        set coord_rot_angle($i) 0
        set coord_offset($i) 0
        set pos($i) 0
     }

     set dpp_ge(coord_rot) [DPP_GE_COOR_ROT_WCS_ROTATION  g68_first_vec g68_second_vec coord_rot_angle coord_offset pos ]

     if { ![info exists g68_first_vec] || ![info exists g68_second_vec] ||\
          ![info exists coord_rot_angle] || ![info exists coord_offset] || ![info exists pos] } {
        return 0
     }

     for { set i 0 } { $i < 3 } { incr i } {
        set dpp_ge(g68_first_vec,$i) $g68_first_vec($i)
        set prev_g68_first_vec($i) $dpp_ge(prev_g68_first_vec,$i)

        set dpp_ge(g68_second_vec,$i) $g68_second_vec($i)
        set prev_g68_second_vec($i) $dpp_ge(prev_g68_second_vec,$i)

        set dpp_ge(coord_offset,$i) $coord_offset($i)
        set prev_coord_offset($i) $dpp_ge(prev_coord_offset,$i)

        set dpp_ge(coord_rot_angle,$i) $coord_rot_angle($i)
        set prev_coord_rot_angle($i) $dpp_ge(prev_coord_rot_angle,$i)
     }

     # if it's auto3d condition, compare the G68 parameters of this time with last time to check if tool axis changes
     if { [string compare $dpp_ge(coord_rot) "NONE"] } {
        VMOV 3 pos mom_pos
        # Compare the G68 parameters to the last ones, if they are same, don't output G68
        if { [VEC3_is_equal g68_first_vec prev_g68_first_vec] &&\
             [VEC3_is_equal g68_second_vec prev_g68_second_vec] &&\
             [VEC3_is_equal coord_offset prev_coord_offset] &&\
             [VEC3_is_equal coord_rot_angle prev_coord_rot_angle] &&\
             [EQ_is_equal $mom_out_angle_pos(0) $mom_prev_out_angle_pos(0)] &&\
             [EQ_is_equal $mom_out_angle_pos(1) $mom_prev_out_angle_pos(1)]} {
           return 0
        } else {
           for { set i 0 } { $i < 3 } { incr i } {
              set dpp_ge(prev_g68_first_vec,$i) $dpp_ge(g68_first_vec,$i)
              set dpp_ge(prev_g68_second_vec,$i) $dpp_ge(g68_second_vec,$i)
              set dpp_ge(prev_coord_offset,$i) $dpp_ge(coord_offset,$i)
              set dpp_ge(prev_coord_rot_angle,$i) $dpp_ge(coord_rot_angle,$i)
           }
           MOM_output_literal "G49"
           MOM_output_literal "G69"
           set dpp_ge(cycle_plane_change) TRUE
           # Recalculate the hole parameters for this hole
           VMOV 3 mom_pos mom_cycle_rapid_to_pos
           VMOV 3 mom_pos mom_cycle_feed_to_pos
           VMOV 3 mom_pos mom_cycle_retract_to_pos
           set mom_cycle_rapid_to_pos(2) [expr $mom_pos(2)+$mom_cycle_rapid_to]
           set mom_cycle_retract_to_pos(2) [expr $mom_pos(2)+$mom_cycle_retract_to]
           set mom_cycle_feed_to_pos(2) [expr $mom_pos(2)+$mom_cycle_feed_to]
           MOM_force Once G_motion G_adjust tap_string X Y Z H F R dwell cycle_step
           set mom_cycle_spindle_axis 2
           return 1
        }
     } else {
        for { set i 0 } { $i < 3 } { incr i } {
           set $dpp_ge(prev_g68_first_vec,$i) 0
           set $dpp_ge(prev_g68_second_vec,$i) 0
           set $dpp_ge(prev_coord_offset,$i) 0
           set $dpp_ge(prev_coord_rot_angle,$i) 0
        }

       # If it's not auto3d condition, restore the kinematics and recalculate mom_pos
        DPP_GE_RESTORE_KINEMATICS
        PB_CMD__convert_point

        MOM_output_literal "G49"
        MOM_output_literal "G69"
        MOM_force Once G_motion G_adjust tap_string X Y Z H F R dwell cycle_step fourth_axis fifth_axis

        VMOV 3 mom_pos mom_cycle_rapid_to_pos
        VMOV 3 mom_pos mom_cycle_feed_to_pos
        VMOV 3 mom_pos mom_cycle_retract_to_pos
        set mom_cycle_rapid_to_pos(2) [expr $mom_pos(2)+$mom_cycle_rapid_to]
        set mom_cycle_retract_to_pos(2) [expr $mom_pos(2)+$mom_cycle_retract_to]
        set mom_cycle_feed_to_pos(2) [expr $mom_pos(2)+$mom_cycle_feed_to]

        MOM_reload_variable -a mom_pos

        return 0
     }
  } else {
     return 0
  }
}


#=============================================================
proc PB_CMD__check_block_cycle_plane_change_to_auto_align_rotary_axis { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

# If G68.2 is output, output G53.1 to auto align rotary axis.

  global dpp_ge

  if {$dpp_ge(toolpath_axis_num)=="3" && $dpp_ge(sys_coord_rotation_output_type)=="SWIVELING" && $dpp_ge(cycle_plane_change)} {
return 1
  } else {
return 0
}


}


#=============================================================
proc PB_CMD__check_block_cycle_rapidtoZ { } {
#=============================================================
# This custom command should return
#   1 : Output block
#   0 : No output
#
# 07-May-15 Jintao - fix PR7162261 and PTP/Hole making issues

  global mom_prev_pos
  global mom_tool_axis
  global mom_operation_type
  global mom_cycle_retract_to
  global mom_cycle_clearance_pos
  global mom_cycle_retract_mode

  if { [info exists mom_cycle_retract_mode] && [string match "AUTO" $mom_cycle_retract_mode] } {
     return 0
  }

  if { [string match "Point to Point" $mom_operation_type] ||\
       [string match "Hole Making" $mom_operation_type] } {
     VEC3_scale mom_cycle_retract_to mom_tool_axis retract_to_pos
     VEC3_add mom_prev_pos retract_to_pos retract_to_pos
     VEC3_sub mom_cycle_clearance_pos retract_to_pos delta_vec
     set dist [VEC3_dot delta_vec mom_tool_axis]
     if { [EQ_is_gt $dist 0.0] } {
        return 1
     } else {
        return 0
     }
  } else {
     return 0
  }
}


#=============================================================
proc PB_CMD__check_block_g68_first_coord_rot { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

# Check if G68 should output and how many times need to output. G68 can output twice at most. In internal
# calculation, always output G68 twice. But if one of the rotation angle is 0, adjust the parameters for G68 and
# just output once.
#
# Output:
#   dpp_ge(coord_rot) - the flag to indicate if it's a 3+2 operation
#   dpp_ge(g68_first_vec,$i) - the first vector coordinate system rotate around
#   dpp_ge(g68_second_vec,$i) - the second vector coordinate system rotate around
#   dpp_ge(coord_offset,$i) - the linear offset G68 should output
#   dpp_ge(coord_rot_angle,$i) - the angles coordinate system rotate around the vectors
#   dpp_ge(coord_offset2,$i) - if the first G68 don't output, assign the linear offset to this array
#
# Return:
#   1 - output the first G68
#   0 - don't output the first G68
#
# Revisions:
#-----------
# 2013-05-27 levi - Initial implementation
#


  global dpp_ge
  global mom_pos
  global mom_out_angle_pos
  global mom_prev_out_angle_pos
  global save_mom_kin_machine_type

  if { $save_mom_kin_machine_type=="5_axis_dual_table" || $save_mom_kin_machine_type=="4_axis_table"} {
    return 0
  }

  if {$dpp_ge(toolpath_axis_num)=="3" && $dpp_ge(sys_coord_rotation_output_type)=="WCS_ROTATION"} {
     if {[string compare $dpp_ge(coord_rot) "NONE"]} {
        if {![EQ_is_equal $dpp_ge(coord_rot_angle,0) 0]} {
           set dpp_ge(coord_offset2,0) 0
           set dpp_ge(coord_offset2,1) 0
           set dpp_ge(coord_offset2,2) 0
           # restore out angle pos to suppress rotary axis output
           if {[string match $dpp_ge(coord_rot) "LOCAL"]} {
              set mom_out_angle_pos(0) $dpp_ge(save_out_angle_pos,0)
              set mom_out_angle_pos(1) $dpp_ge(save_out_angle_pos,1)
           }
           set mom_prev_out_angle_pos(0) $mom_out_angle_pos(0)
           set mom_prev_out_angle_pos(1) $mom_out_angle_pos(1)

           # Generate rotary axis angle, but don't output to file. Hence, if tool axis doesn't change, rotary axis
           # won't output. It has the same effect as MOM_disable_address under 3+2 condition.
           MOM_do_template three_plus_two_suppress CREATE
           return 1
        } else {
           set dpp_ge(coord_offset2,0) $dpp_ge(coord_offset,0)
           set dpp_ge(coord_offset2,1) $dpp_ge(coord_offset,1)
           set dpp_ge(coord_offset2,2) $dpp_ge(coord_offset,2)
           return 0
        }
     } else {
        return 0
     }
  } else {
return 0
  }
}


#=============================================================
proc PB_CMD__check_block_g68_second_coord_rot { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

# Check if the second G68 should output.

  global dpp_ge
  global mom_out_angle_pos
  global mom_prev_out_angle_pos
  global save_mom_kin_machine_type

  if { $save_mom_kin_machine_type=="5_axis_dual_table" || $save_mom_kin_machine_type=="4_axis_table"} {
    return 0
  }

  if {$dpp_ge(toolpath_axis_num)=="3" && $dpp_ge(sys_coord_rotation_output_type)=="WCS_ROTATION" && [string compare $dpp_ge(coord_rot) "NONE"]} {
     if {![EQ_is_equal $dpp_ge(coord_rot_angle,1) 0] || ![EQ_is_equal $dpp_ge(coord_offset2,0) 0] || ![EQ_is_equal $dpp_ge(coord_offset2,1) 0] || ![EQ_is_equal $dpp_ge(coord_offset2,2) 0]} {
        # restore out angle pos to suppress rotary axis output
        if {[string match $dpp_ge(coord_rot) "LOCAL"]} {
           set mom_out_angle_pos(0) $dpp_ge(save_out_angle_pos,0)
           set mom_out_angle_pos(1) $dpp_ge(save_out_angle_pos,1)
        }
        set mom_prev_out_angle_pos(0) $mom_out_angle_pos(0)
        set mom_prev_out_angle_pos(1) $mom_out_angle_pos(1)

        # Generate rotary axis angle, but don't output to file. Hence, if tool axis doesn't change, rotary axis
        # won't output. It has the same effect as MOM_disable_address under 3+2 condition.
        MOM_do_template three_plus_two_suppress CREATE
        return 1
     } else {
        return 0
     }
  } else {
     return 0
  }

}


#=============================================================
proc PB_CMD__check_block_is_in_transition_path { } {
#=============================================================
# This custom command should return
#   1 : Output
#   0 : No output

   global mom_logname

   if { [info exists ::mom_sys_in_transition_path] && $::mom_sys_in_transition_path } {
 return 1
   } else {
 return 0
   }
}


#=============================================================
proc PB_CMD__check_block_mill_default_condition { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output
# Deal with the default condition

global mom_motion_type
global mom_machine_mode
global mom_current_motion

global dpp_ge

if { ![string compare $mom_machine_mode "MILL"] } {
   if { ($dpp_ge(sys_tcp_tool_axis_output_mode) == "VECTOR" && $dpp_ge(toolpath_axis_num) == "5") } {
      return 0

      } else {
             return 1
             }

   } else {
          return 0
          }
}


#=============================================================
proc PB_CMD__check_block_mill_work_plane { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

global mom_machine_mode

if { ![string compare $mom_machine_mode "MILL"] } {
   return 1
   }

return 0
}


#=============================================================
proc PB_CMD__check_block_next_tool_select { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output
#
# Check if current tool is the last tool, if so, don't preselect next tool.

  global mom_next_tool_status

  if { $mom_next_tool_status == "FIRST" } {
     return 0
  } else {
     return 1
  }
}


#=============================================================
proc PB_CMD__check_block_output_activate_tool_center_point_control { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

global dpp_ge

if { [string match "5" $dpp_ge(toolpath_axis_num)] } {
   MOM_disable_address G_adjust

   return 1
   }

return 0
}


#=============================================================
proc PB_CMD__check_block_output_cancel_dynamic_work_offset_first_move { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

global mom_rotary_delta_4th
global mom_rotary_delta_5th

global dpp_ge

if { [string match "5" $dpp_ge(prev_toolpath_axis_num)] } {
   return 0
   }

if { [string match "5" $dpp_ge(toolpath_axis_num)] || \
     ([string match "3" $dpp_ge(toolpath_axis_num)] && [expr $mom_rotary_delta_4th + $mom_rotary_delta_5th] != 0.0) } {
   return 1
   }

return 0
}


#=============================================================
proc PB_CMD__check_block_output_clamp_codes_first_move { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

global mom_rotary_delta_4th
global mom_rotary_delta_5th

global dpp_ge

if { [string match "5" $dpp_ge(toolpath_axis_num)] || \
     ([string match "3" $dpp_ge(toolpath_axis_num)] && [expr $mom_rotary_delta_4th + $mom_rotary_delta_5th] == 0.0) } {
   return 0
   }

return 1
}


#=============================================================
proc PB_CMD__check_block_output_clamp_codes_initial_move { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

global dpp_ge

if { [string match "5" $dpp_ge(toolpath_axis_num)] } {
   return 0
   }

return 1
}


#=============================================================
proc PB_CMD__check_block_output_codes_when_tool_axis_change_first_move { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

global mom_rotary_delta_4th
global mom_rotary_delta_5th

global dpp_ge

if { [string match "5" $dpp_ge(toolpath_axis_num)] || \
     ([string match "3" $dpp_ge(toolpath_axis_num)] && [expr $mom_rotary_delta_4th + $mom_rotary_delta_5th] != 0.0) } {
   return 1
   }

return 0
}


#=============================================================
proc PB_CMD__check_block_output_coolant_off { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

global mom_coolant_status
global mom_sys_coolant_code
global mom_next_oper_has_tool_change
global mom_current_oper_is_last_oper_in_program

if { [info exists mom_coolant_status] && [string match "TAP" $mom_coolant_status] } {
   set mom_sys_coolant_code(OFF) "74"
   }

if { [string match "YES" $mom_next_oper_has_tool_change] || \
     [string match "YES" $mom_current_oper_is_last_oper_in_program] } {
   return 1
   }

return 0
}


#=============================================================
proc PB_CMD__check_block_output_cycle_retract { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

global mom_operation_type
global mom_cycle_retract_mode
global mom_cycle_retract_to_pos

global save_mom_pos

if { [string match "Point to Point" $mom_operation_type] } {
   if { [string match "AUTO" $mom_cycle_retract_mode] } {
      set mom_cycle_retract_to_pos(2) $save_mom_pos(2)
      }
   }

MOM_do_template cycle_off

MOM_cycle_off

MOM_force once G_motion Z

return 1
}


#=============================================================
proc PB_CMD__check_block_output_initial_move_rotation_first_move { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

global mom_tool_axis
global mom_out_angle_pos
global mom_rotary_delta_4th
global mom_rotary_delta_5th

global dpp_ge

if { [string match "5" $dpp_ge(toolpath_axis_num)] || \
     ([string match "3" $dpp_ge(toolpath_axis_num)] && [expr $mom_rotary_delta_4th + $mom_rotary_delta_5th] != 0.0) } {

   if { $mom_tool_axis(0) == 0.0 && $mom_tool_axis(1) == 0.0 && $mom_tool_axis(2) == 1.0 } {
      set mom_out_angle_pos(0) 0.0
      set mom_out_angle_pos(1) 0.0
      }

   return 1
   }

return 0
}


#=============================================================
proc PB_CMD__check_block_output_initial_move_rotation_initial_move { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

global mom_tool_axis
global mom_out_angle_pos

if { ($mom_tool_axis(0) == 0.0 && $mom_tool_axis(1) == 0.0 && $mom_tool_axis(2) == 1.0) && \
     $mom_out_angle_pos(1) != 0.0 } {
   set mom_out_angle_pos(0) 0.0
   set mom_out_angle_pos(1) 0.0
   }

return 1
}


#=============================================================
proc PB_CMD__check_block_output_return_home_bc_first_tool { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

global first_tool_status

if { ![info exists first_tool_status] } {
   return 1
   }

return 0
}


#=============================================================
proc PB_CMD__check_block_output_rotary_before { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output
# Decide if rotary move should be output eariler.
#
# 05-09-2013 levi - Add "&& $dpp_ge(coord_rot)=="LOCAL"" in judgement, because under local csys condition, the rotary angles
# are stored in mom_init_pos, and this array won't be unset at the end of path. So even the next operation is under auto3d
# condition this array also exists.

global mom_pos
global mom_init_pos
global mom_sys_leader
global mom_out_angle_pos
global mom_prev_out_angle_pos
global mom_kin_4th_axis_leader
global mom_kin_5th_axis_leader
global mom_kin_4th_axis_max_limit
global mom_kin_5th_axis_max_limit
global mom_kin_4th_axis_min_limit
global mom_kin_5th_axis_min_limit
global mom_kin_4th_axis_direction
global mom_kin_5th_axis_direction

global dpp_ge
global save_mom_kin_machine_type

# save current out angle pos, will be used to suppress rotary axis in 3+2 mode under G68 condition
set dpp_ge(save_out_angle_pos,0) $mom_out_angle_pos(0)
set dpp_ge(save_out_angle_pos,1) $mom_out_angle_pos(1)

# Should output rotary axis before when multi-axis mode output or G68 output.
if { $dpp_ge(toolpath_axis_num) == "3" && $dpp_ge(sys_coord_rotation_output_type) == "SWIVELING" } {
   if { $dpp_ge(coord_rot) == "NONE" } {
      return 1

      } else {
             return 0
             }

   } elseif { $dpp_ge(toolpath_axis_num) == "3" && $dpp_ge(sys_coord_rotation_output_type) == "WCS_ROTATION" } {
            if { ![info exists mom_prev_out_angle_pos(0)] } {
               set mom_prev_out_angle_pos(0)    0
               set mom_prev_out_angle_pos(1)    0
               }

            # To avoid the can't save bug of pb, initialize the local variable. Just for sim05 vnc!
            for { set i 0 } { $i < 3 } { incr i } {
                set g68_first_vec($i) 0
                set g68_second_vec($i) 0
                set g68_coord_rotation($i) 0
                set offset($i) 0
                set pos($i) 0
                }

            set dpp_ge(coord_rot) [DPP_GE_COOR_ROT_WCS_ROTATION  g68_first_vec g68_second_vec g68_coord_rotation offset pos]

            if { ![info exists g68_first_vec] || ![info exists g68_second_vec] || ![info exists g68_coord_rotation] || ![info exists offset] || ![info exists pos] } {
               return 0
               }

            #Pop up warning message when user wants to output G68 under local csys on a machine with table.
            if { $dpp_ge(coord_rot) == "LOCAL" && [string match "*table*" $save_mom_kin_machine_type] } {
               MOM_abort "***Don't use local csys to output G68 on a machine with table. Please attach your operation to fixture offset coordinate system.***"
               } elseif { $save_mom_kin_machine_type == "5_axis_dual_table" || $save_mom_kin_machine_type == "4_axis_table" } {
                        DPP_GE_RESTORE_KINEMATICS

                        set dpp_ge(coord_rot)  "NONE"

                        return 1
                        }

            for { set i 0 } { $i < 3 } { incr i } {
                set dpp_ge(g68_first_vec,$i) $g68_first_vec($i)
                set dpp_ge(g68_second_vec,$i) $g68_second_vec($i)
                set dpp_ge(coord_offset,$i) $offset($i)
                set dpp_ge(coord_rot_angle,$i) $g68_coord_rotation($i)

                set dpp_ge(prev_g68_first_vec,$i) $g68_first_vec($i)
                set dpp_ge(prev_g68_second_vec,$i) $g68_second_vec($i)
                set dpp_ge(prev_coord_offset,$i) $offset($i)
                set dpp_ge(prev_coord_rot_angle,$i) $g68_coord_rotation($i)

                set mom_pos($i) $pos($i)
                }

            MOM_reload_variable -a mom_pos

            if { [info exists mom_init_pos(3)] && $dpp_ge(coord_rot) == "LOCAL" } {
               set mom_out_angle_pos(0) [ROTSET $mom_init_pos(3) $mom_prev_out_angle_pos(0) $mom_kin_4th_axis_direction $mom_kin_4th_axis_leader mom_sys_leader(fourth_axis) $mom_kin_4th_axis_min_limit $mom_kin_4th_axis_max_limit]
               }

            if { [info exists mom_init_pos(4)] && $dpp_ge(coord_rot) == "LOCAL" } {
               set mom_out_angle_pos(1) [ROTSET $mom_init_pos(4) $mom_prev_out_angle_pos(1) $mom_kin_5th_axis_direction $mom_kin_5th_axis_leader mom_sys_leader(fifth_axis) $mom_kin_5th_axis_min_limit $mom_kin_5th_axis_max_limit]
               }

            return 1

            } else {
                   return 1
                   }
}


#=============================================================
proc PB_CMD__check_block_output_spindle_rpm_first_move { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

global mom_spindle_speed

global save_mom_spindle_speed

if { ![info exists save_mom_spindle_speed] } { set save_mom_spindle_speed 0.0 }

if { ![EQ_is_equal $mom_spindle_speed $save_mom_spindle_speed] } {
   return 1
   }

return 0
}


#=============================================================
proc PB_CMD__check_block_output_unclamp_codes_end_of_program { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

global dpp_ge

if { [string match "3" $dpp_ge(toolpath_axis_num)] } {
   return 1
   }

return 0
}


#=============================================================
proc PB_CMD__check_block_output_unclamp_codes_first_move { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

global mom_rotary_delta_4th
global mom_rotary_delta_5th

global dpp_ge

if { [string match "5" $dpp_ge(prev_toolpath_axis_num)] } {
   return 0
   }

if { [string match "5" $dpp_ge(toolpath_axis_num)] || \
     ([string match "3" $dpp_ge(toolpath_axis_num)] && [expr $mom_rotary_delta_4th + $mom_rotary_delta_5th] != 0.0) } {
   return 1
   }

return 0
}


#=============================================================
proc PB_CMD__check_block_output_unclamp_codes_initial_move { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

global dpp_ge

if { [string match "5" $dpp_ge(prev_toolpath_axis_num)] } {
   return 0
   }

return 1
}


#=============================================================
proc PB_CMD__check_block_position_after_cycle_plane_change { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

  global dpp_ge

  # If tool axis changed between holes, position tool to R point before calling cycle
   if { $dpp_ge(cycle_plane_change) } {
      MOM_force Once tap_string F R dwell cycle_step
      return 1
   } else {
      return 0
   }
}


#=============================================================
proc PB_CMD__check_block_return_to_reference_point { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output
#
# If next operation has tool change or it's the last operation, go to reference point, turn off
# spindle and coolant. Otherwise, don't do these.
#
# 2013-06-07 levi - Initial version.
# 2013-10-16 levi - Set the rotary angles to 0 and reload the variables.

global mom_pos
global mom_prev_pos
global mom_out_angle_pos
global mom_prev_out_angle_pos
global mom_next_oper_has_tool_change
global mom_current_oper_is_last_oper_in_program

if { ([info exists mom_next_oper_has_tool_change] && $mom_next_oper_has_tool_change == "YES") || ([info exists mom_current_oper_is_last_oper_in_program] && $mom_current_oper_is_last_oper_in_program == "YES") } {
   set mom_out_angle_pos(0) 0
   set mom_out_angle_pos(1) 0

   set mom_prev_out_angle_pos(0) 0.0
   set mom_prev_out_angle_pos(1) 0.0

   set mom_pos(3) 0.0
   set mom_pos(4) 0.0

   set mom_prev_pos(3) 0.0
   set mom_prev_pos(4) 0.0

   MOM_reload_variable -a mom_out_angle_pos
   MOM_reload_variable -a mom_prev_out_angle_pos
   MOM_reload_variable -a mom_pos
   MOM_reload_variable -a mom_prev_pos

   return 1

   } else {
          return 0
          }
}


#=============================================================
proc PB_CMD__check_block_running_post_oper_path { } {
#=============================================================
# This custom command should return
#   1 : Output
#   0 : No output

   global mom_logname

   if { [info exists ::mom_post_oper_path] && $::mom_post_oper_path } {
      OPERATOR_MSG_debug "==> Running MOM_post_oper_path, skip [info level -1]"
      MOM_enable_address N
      CLOSE_files
 return 1
   }

 return 0
}


#=============================================================
proc PB_CMD__check_block_subprogram_output_off { } {
#=============================================================
# Flag of subprogram output for patterned operations is determined in PB_CMD__pattern_set_csys_start
#
# This custom command should return
#   1 : Output
#   0 : No output

   global mom_logname

   if { [info exists ::PATTERN_OUTPUT(SUB)] && $::PATTERN_OUTPUT(SUB) } {
 return 0
   } else {
 return 1
   }
}


#=============================================================
proc PB_CMD__check_block_subprogram_output_on { } {
#=============================================================
# This condition command can be called by MOM_initial_move,
# MOM_first_move & MOM_end_of_program, or any event applicable,
# to determine if subprogram output has been enabled.
#
# Flag of subprogram output for patterned operations is determined
# in PB_CMD__pattern_set_csys_start
#
# This custom command should return
#   1 : Output
#   0 : No output

   global mom_logname

   if { [info exists ::PATTERN_OUTPUT(SUB)] && $::PATTERN_OUTPUT(SUB) } {
 return 1
   } else {
 return 0
   }
}


#=============================================================
proc PB_CMD__check_block_swiveling_coord_rot { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

# Check if G68.2 should output.
#
# Output:
#   dpp_ge(coord_rot) - the flag to indicate if it's a 3+2 operation
#   dpp_ge(coord_offset,$i) - the linear offset G68.2 should output
#   dpp_ge(coord_rot_angle,$i) - the angles coordinate system rotate around the coordinate system axis
#   dpp_ge(prev_coord_rot_angle,$i) - Record the rotation angles for this time as previous angles. Used to check if cycle plane changes.
#
# Return:
#   1 - output G68.2
#   0 - don't output G68.2
#
# Revisions:
#-----------
# 2013-05-27 levi - Initial implementation
#

  global dpp_ge
  global mom_pos

  if { $dpp_ge(toolpath_axis_num) == "3" && $dpp_ge(sys_coord_rotation_output_type) == "SWIVELING" } {
     set dpp_ge(coord_rot) [DPP_GE_COOR_ROT "ZXZ" angle offset pos]
     if { [string compare "NONE" $dpp_ge(coord_rot)] } {
        for { set i 0 } { $i < 3 } { incr i } {
           if { [info exists offset] } {
              set dpp_ge(coord_offset,$i) $offset($i)
           }
           if { [info exists angle] } {
              set dpp_ge(coord_rot_angle,$i) $angle($i)
              set dpp_ge(prev_coord_rot_angle,$i) $angle($i)
           }
           if { [info exists pos] } {
              set mom_pos($i) $pos($i)
           }
        }
        MOM_reload_variable -a mom_pos

       # Generate rotary axis angle, but don't output to file. Hence, if tool axis doesn't change, rotary axis
       # won't output. It has the same effect as MOM_disable_address under 3+2 condition.
        MOM_do_template three_plus_two_suppress CREATE

        return 1
     } else {
        return 0
     }
  } else {
     return 0
  }
}


#=============================================================
proc PB_CMD__check_block_swiveling_coord_rot_first_move { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

# Check if G68.2 should output.
#
# Output:
#   dpp_ge(coord_rot) - the flag to indicate if it's a 3+2 operation
#   dpp_ge(coord_offset,$i) - the linear offset G68.2 should output
#   dpp_ge(coord_rot_angle,$i) - the angles coordinate system rotate around the coordinate system axis
#   dpp_ge(prev_coord_rot_angle,$i) - Record the rotation angles for this time as previous angles. Used to check if cycle plane changes.
#
# Return:
#   1 - output G68.2
#   0 - don't output G68.2

global mom_pos
global mom_rotary_delta_4th
global mom_rotary_delta_5th

global dpp_ge

if { $dpp_ge(toolpath_axis_num) == "3" && $dpp_ge(sys_coord_rotation_output_type) == "SWIVELING" } {
   set dpp_ge(coord_rot) [DPP_GE_COOR_ROT "ZXZ" angle offset pos]

   if { [string compare "NONE" $dpp_ge(coord_rot)] } {
      for { set i 0 } { $i < 3 } { incr i } {
          if { [info exists offset] } {
             set dpp_ge(coord_offset,$i) $offset($i)
             }

          if { [info exists angle] } {
             set dpp_ge(coord_rot_angle,$i) $angle($i)
             set dpp_ge(prev_coord_rot_angle,$i) $angle($i)
             }

          if { [info exists pos] } {
             set mom_pos($i) $pos($i)
             }
          }

      MOM_reload_variable -a mom_pos

      if { [expr $mom_rotary_delta_4th + $mom_rotary_delta_5th] == 0.0 } {
         return 0
         }

      return 1

      } else {
             return 0
             }

   } else {
          return 0
          }
}


#=============================================================
proc PB_CMD__check_block_swiveling_coord_rot_initial_move { } {
#=============================================================
# This custom command should return
#   1 : Output BLOCK
#   0 : No output

# Check if G68.2 should output.
#
# Output:
#   dpp_ge(coord_rot) - the flag to indicate if it's a 3+2 operation
#   dpp_ge(coord_offset,$i) - the linear offset G68.2 should output
#   dpp_ge(coord_rot_angle,$i) - the angles coordinate system rotate around the coordinate system axis
#   dpp_ge(prev_coord_rot_angle,$i) - Record the rotation angles for this time as previous angles. Used to check if cycle plane changes.
#
# Return:
#   1 - output G68.2
#   0 - don't output G68.2

global mom_pos

global dpp_ge

if { $dpp_ge(toolpath_axis_num) == "3" && $dpp_ge(sys_coord_rotation_output_type) == "SWIVELING" } {
   set dpp_ge(coord_rot) [DPP_GE_COOR_ROT "ZXZ" angle offset pos]

   if { [string compare "NONE" $dpp_ge(coord_rot)] } {
      for { set i 0 } { $i < 3 } { incr i } {
          if { [info exists offset] } {
             set dpp_ge(coord_offset,$i) $offset($i)
             }

          if { [info exists angle] } {
             set dpp_ge(coord_rot_angle,$i) $angle($i)
             set dpp_ge(prev_coord_rot_angle,$i) $angle($i)
             }

          if { [info exists pos] } {
             set mom_pos($i) $pos($i)
             }
          }

      MOM_reload_variable -a mom_pos

      return 1

      } else {
             return 0
             }

   } else {
          return 0
          }
}


#=============================================================
proc PB_CMD__choose_preferred_solution { } {
#=============================================================
# ==> Do not rename this command!
#
#  This command will recompute rotary angles using the alternate solution
#  of a 5-axis motion based on the setting of "mom_preferred_zone_flag"
#  as the preferred delimiter.  The choices are:
#
#    [XPLUS | XMINUS | YPLUS | YMINUS | FOURTH | FIFTH]
#
#
#  => This command may be attached to Rapid Move or Cycle Plane Change
#     to influence the solution of the rotary axes.
#  => Initial rotary angle can be influenced by using a "Rotate" UDE.
#  => May need to recompute FRN, since length of travel may change.
#
#-------------------------------------------------------------
#<04-24-2014 gsl> Attempt to resolve PR#6738915
#<07-13-2015 gsl> Reworked logic for FOURTH & FIFTH cases
#
#return

#----------------------------------------------------------
# Preferred zone flag can be set via an UDE or other means.
#
#   EVENT preferred_solution
#   {
#     UI_LABEL "Preferred Solution"
#     PARAM choose_preferred_zone
#     {
#        TYPE b
#        DEFVAL "TRUE"
#        UI_LABEL "Choose Preferred Zone"
#     }
#     PARAM preferred_zone_flag
#     {
#        TYPE o
#        DEFVAL "YPLUS"
#        OPTIONS "XPLUS","XMINUS","YPLUS","YMINUS","FOURTH","FIFTH"
#        UI_LABEL "Preferred Zone"
#     }
#   }

if { [info exists ::mom_preferred_zone_flag] } {
   # Only handle Rapid & Cycles for the time being,
   # user may add other cases as desired.
   if { [string compare "RAPID" $::mom_motion_type] && \
        [string compare "CYCLE" $::mom_motion_type] } {
      return
      }

   if { ![info exists ::mom_prev_out_angle_pos] } {
      array set ::mom_prev_out_angle_pos [array get ::mom_out_angle_pos]

      MOM_reload_variable -a mom_prev_out_angle_pos

      return
      }

   set co "$::mom_sys_control_out"
   set ci "$::mom_sys_control_in"

   set __use_alternate 0

   switch $::mom_preferred_zone_flag {
                                 XPLUS {
                                       if { !([EQ_is_gt $::mom_pos(0) 0.0] || [EQ_is_zero $::mom_pos(0)]) } {
                                          set __use_alternate 1
                                          }
                                       }
                                XMINUS {
                                       if { !([EQ_is_le $::mom_pos(0) 0.0]) } {
                                          set __use_alternate 1
                                          }
                                       }
                                 YPLUS {
                                       if { !([EQ_is_gt $::mom_pos(1) 0.0] || [EQ_is_zero $::mom_pos(1)]) } {
                                          set __use_alternate 1
                                          }
                                       }
                                YMINUS {
                                       if { !([EQ_is_le $::mom_pos(1) 0.0]) } {
                                          set __use_alternate 1
                                          }
                                       }
                                FOURTH {
                                       set del4 [expr abs( $::mom_out_angle_pos(0) - $::mom_prev_out_angle_pos(0) )]

                                       VMOV 5 ::mom_alt_pos ::mom_pos

                                       set out_angle_4th [ROTSET $::mom_pos(3) $::mom_prev_out_angle_pos(0) $::mom_kin_4th_axis_direction\
                                                                 $::mom_kin_4th_axis_leader ::mom_sys_leader(fourth_axis)\
                                                                 $::mom_kin_4th_axis_min_limit $::mom_kin_4th_axis_max_limit]

                                       set del4a [expr abs( $out_angle_4th - $::mom_prev_out_angle_pos(0) )]

                                       if [expr $del4 > $del4a] {
                                          set __use_alternate 1
                                          }
                                       }
                                 FIFTH {
                                       set del5 [expr abs( $::mom_out_angle_pos(1) - $::mom_prev_out_angle_pos(1) )]

                                       VMOV 5 ::mom_alt_pos ::mom_pos

                                       set out_angle_5th [ROTSET $::mom_pos(4) $::mom_prev_out_angle_pos(1) $::mom_kin_5th_axis_direction\
                                                                 $::mom_kin_5th_axis_leader ::mom_sys_leader(fifth_axis)\
                                                                 $::mom_kin_5th_axis_min_limit $::mom_kin_5th_axis_max_limit]

                                       set del5a [expr abs( $out_angle_5th - $::mom_prev_out_angle_pos(1) )]

                                       if [expr $del5 > $del5a] {
                                          set __use_alternate 1
                                          }
                                       }
                               default {
                                       CATCH_WARNING "$co Preferred delimiter \"$::mom_preferred_zone_flag\" is not available! $ci"
                                       }
                                     }

   # Recompute output when needed
   if { $__use_alternate } {
      set a4 $::mom_out_angle_pos(0)
      set a5 $::mom_out_angle_pos(1)

      VMOV 5 ::mom_alt_pos ::mom_pos

      set ::mom_out_angle_pos(0) [ROTSET $::mom_pos(3) $::mom_prev_out_angle_pos(0) $::mom_kin_4th_axis_direction\
                                         $::mom_kin_4th_axis_leader ::mom_sys_leader(fourth_axis)\
                                         $::mom_kin_4th_axis_min_limit $::mom_kin_4th_axis_max_limit]
      set ::mom_out_angle_pos(1) [ROTSET $::mom_pos(4) $::mom_prev_out_angle_pos(1) $::mom_kin_5th_axis_direction\
                                         $::mom_kin_5th_axis_leader ::mom_sys_leader(fifth_axis)\
                                         $::mom_kin_5th_axis_min_limit $::mom_kin_5th_axis_max_limit]

      MOM_reload_variable -a mom_out_angle_pos
      MOM_reload_variable -a mom_pos

      set msg "$co Use alternate solution : $::mom_preferred_zone_flag \
                   ($a4 / $a5) -> ($::mom_out_angle_pos(0) / $::mom_out_angle_pos(1)) $ci"

      CATCH_WARNING $msg
      }

   # Recompute output coords for cycles
   if { ![info exists ::mom_sys_cycle_after_initial] } {
      set ::mom_sys_cycle_after_initial "FALSE"
      }

   if { [string match "CYCLE" $::mom_motion_type] } {
      if { [string match "initial_move" $::mom_motion_event] } {
         set ::mom_sys_cycle_after_initial "TRUE"

         return
         }

      if { [string match "TRUE" $::mom_sys_cycle_after_initial] } {
         set ::mom_pos(0) [expr $::mom_pos(0) - $::mom_cycle_rapid_to * $::mom_tool_axis(0)]
         set ::mom_pos(1) [expr $::mom_pos(1) - $::mom_cycle_rapid_to * $::mom_tool_axis(1)]
         set ::mom_pos(2) [expr $::mom_pos(2) - $::mom_cycle_rapid_to * $::mom_tool_axis(2)]
         }

      set ::mom_sys_cycle_after_initial "FALSE"

      if { [string match "Table" $::mom_kin_4th_axis_type] } {
         #"mom_spindle_axis" would have incorporated the direction of head attachment already.
         if [info exists ::mom_spindle_axis] {
            VMOV 3 ::mom_spindle_axis ::mom_sys_spindle_axis

            } else {
                   VMOV 3 ::mom_kin_spindle_axis ::mom_sys_spindle_axis
                   }

         } elseif { [string match "Table" $::mom_kin_5th_axis_type] } {
                  VMOV 3 ::mom_tool_axis vec

                  switch $::mom_kin_4th_axis_plane {
                                                  XY {
                                                     set vec(2) 0.0
                                                     }
                                                  ZX {
                                                     set vec(1) 0.0
                                                     }
                                                  YZ {
                                                     set vec(0) 0.0
                                                     }
                                                   }

                  #Reworked logic to prevent potential error
                  set len [VEC3_mag vec]
                  if { [EQ_is_gt $len 0.0] } {
                     VEC3_unitize vec ::mom_sys_spindle_axis

                     } else {
                            set ::mom_sys_spindle_axis(0) 0.0
                            set ::mom_sys_spindle_axis(1) 0.0
                            set ::mom_sys_spindle_axis(2) 1.0
                            }

                  } else {
                         VMOV 3 ::mom_tool_axis ::mom_sys_spindle_axis
                         }

      set ::mom_cycle_feed_to_pos(0)    [expr $::mom_pos(0) + $::mom_cycle_feed_to    * $::mom_sys_spindle_axis(0)]
      set ::mom_cycle_feed_to_pos(1)    [expr $::mom_pos(1) + $::mom_cycle_feed_to    * $::mom_sys_spindle_axis(1)]
      set ::mom_cycle_feed_to_pos(2)    [expr $::mom_pos(2) + $::mom_cycle_feed_to    * $::mom_sys_spindle_axis(2)]

      set ::mom_cycle_rapid_to_pos(0)   [expr $::mom_pos(0) + $::mom_cycle_rapid_to   * $::mom_sys_spindle_axis(0)]
      set ::mom_cycle_rapid_to_pos(1)   [expr $::mom_pos(1) + $::mom_cycle_rapid_to   * $::mom_sys_spindle_axis(1)]
      set ::mom_cycle_rapid_to_pos(2)   [expr $::mom_pos(2) + $::mom_cycle_rapid_to   * $::mom_sys_spindle_axis(2)]

      set ::mom_cycle_retract_to_pos(0) [expr $::mom_pos(0) + $::mom_cycle_retract_to * $::mom_sys_spindle_axis(0)]
      set ::mom_cycle_retract_to_pos(1) [expr $::mom_pos(1) + $::mom_cycle_retract_to * $::mom_sys_spindle_axis(1)]
      set ::mom_cycle_retract_to_pos(2) [expr $::mom_pos(2) + $::mom_cycle_retract_to * $::mom_sys_spindle_axis(2)]
      }
   }
}


#=============================================================
proc PB_CMD__config_post_options { } {
#=============================================================
# <PB v10.03>
# This command should be called by Start-of-Program event;
# it enables users to set options (not via UI) that would
# affect the behavior and output of this post.
#
# Comment out next line to activate this command
return

# <PB v10.03>
# - Feed mode for RETRACT motion has been handled as RAPID,
#   next option enables users to treat RETRACT as CONTOURing.

if { ![info exists ::mom_sys_retract_feed_mode] } {
   set ::mom_sys_retract_feed_mode "CONTOUR"
   }
}


#=============================================================
proc PB_CMD__convert_point { } {
#=============================================================
# This command converts global MCS goto to mom_pos & select the preferred solution
#
# Nov-27-2018 gsl - New
# Dec-13-2018 gsl - Reload mom_pos & mom_alt_pos
#
   if { [MOM_convert_point ::mom_mcs_goto ::mom_tool_axis] } {
      set idx -1
      foreach pos $::mom_result {
         set ::mom_pos([incr idx]) $pos
      }
      set idx -1
      foreach pos $::mom_result1 {
         set ::mom_alt_pos([incr idx]) $pos
      }

      MOM_reload_variable -a mom_pos
      MOM_reload_variable -a mom_alt_pos

     # Choose preferred solution per previous 4th or 5th angle
     # set ::mom_preferred_zone_flag FIFTH
      set ::mom_preferred_zone_flag FOURTH

      PB_CMD__choose_preferred_solution

      unset ::mom_preferred_zone_flag
   }
}


#=============================================================
proc PB_CMD__define_debug_msg { } {
#=============================================================
# This special command can be executed at Start of Program to facilitate
# the debugging capability of posts created by Post Builder.
# ==> DO NOT add or call it in any event or command!
# ==> DO NOT change its name!
#
# => Variable "PB_POST__debug" can be set true (1) to activate debug.
#
# => Signature of PB_POST__debug_msg { args } command should not be changed.
#    Calls to this command are embedded in all handlers generated by Post Builder.
#    *** Do Not call it in any before-output commands!!!
#    *** Do Not use it in utility commands e.g. CALLED_BY
#    This command can be called in other custom commands for debugging purpose.
#
# => Contents of debug messages (below) can be configured as desired by the users.
#
#-------------------------------------------------------------
# 01-17-2014 gsl - Initial version (v902)
#
#  ||||||||||||||||||||
#  VVVVVVVVVVVVVVVVVVVV
set PB_POST__debug 0

if $PB_POST__debug {
   uplevel #0 {
      proc PB_POST__debug_msg { args } {
         MOM_output_literal ">>> in [info level -1] - [join $args]"

         #+++++++++++++++++++++++++++++++++++++++++++++++++++
         # Configure the details of variables to be observed
         #
         if [info exists ::mom_out_angle_pos(1)] {
            MOM_output_literal ">>>>> mom_out_angle_pos(1): $::mom_out_angle_pos(1)"
            }
         #+++++++++++++++++++++++++++++++++++++++++++++++++++
         }
      }

   } else {
          uplevel #0 {
             proc PB_POST__debug_msg { args } {}
             }
          }
}


#=============================================================
proc PB_CMD__handle_end_of_subop_path { } {
#=============================================================
# 10/31/2018 gsl - This command can be called by MOM_end_of_subop_path handler.
#
   global mom_move_type
   global mom_move_type_name ;# <-- not reliable!
   global mom_RTCP ;# ON/OFF

   set subop [GET_SUBOP_MOVE_NAME $::mom_move_type]

   OPERATOR_MSG_debug ">>> End of subop >$subop< $::mom_move_type"

   switch "$subop" {
      "Tool_Change_Container" {
        # Remove info of tool change position when done.
         UNSET_VARS ::mom_sys_interop_tool_change_pos
      }
      "Tool_Change_Position" {
        # Save tool change position info
         if { $::mom_interop_has_tool_change_container } {
            VMOV 5 ::mom_pos ::mom_sys_interop_tool_change_pos
         }
      }
      "Rotary_Tool_Center_Point_On" {
      }
      "Rotary_Tool_Center_Point_Off" {
      }
      "Move_to_Machine_Position" {
      }
      "Rotary_Point_Vector_Move" {
      }
      default {
      }
   }
}


#=============================================================
proc PB_CMD__handle_start_of_subop_path { } {
#=============================================================
# 10/31/2018 gsl - This command can be called by MOM_start_of_subop_path handler.
#
   global mom_move_type
   global mom_move_type_name
   global mom_RTCP ;# ON/OFF

   global mom_tool_change_status

   set subop [GET_SUBOP_MOVE_NAME $::mom_move_type]

   OPERATOR_MSG_debug ">>> Start of subop >$subop< $::mom_move_type"

   switch "$subop" {
      "Rotary_Tool_Center_Point_On" {
      }
      "Rotary_Tool_Center_Point_Off" {
      }
      "Move_to_Machine_Position" {
      }
      "Rotary_Point_Vector_Move" {
      }
      default {
      }
   }
}


#=============================================================
proc PB_CMD__interop_done_tool_change { } {
#=============================================================
   set ::mom_sys_tool_change_done 1
   set ::mom_sys_in_transition_path 0
}


#=============================================================
proc PB_CMD__interop_end_subop_path { } {
#=============================================================
# 10/31/2018 gsl - This command can be called by MOM_end_of_subop_path handler.
#
   global mom_move_type
   global mom_move_type_name ;# <-- not reliable!
   global mom_RTCP ;# ON/OFF

   set subop [GET_SUBOP_MOVE_NAME $::mom_move_type]

   OPERATOR_MSG_debug ">>> End of subop >$subop< $::mom_move_type"

   switch "$subop" {
      "Tool_Change_Container" {
        # Remove info of tool change position when done.
         UNSET_VARS ::mom_sys_interop_tool_change_pos
      }
      "Tool_Change_Position" {
        # Save tool change position info
         if { $::mom_interop_has_tool_change_container } {
            VMOV 5 ::mom_pos ::mom_sys_interop_tool_change_pos
         }
      }
      "Rotary_Tool_Center_Point_On" {
      }
      "Rotary_Tool_Center_Point_Off" {
      }
      "Move_to_Machine_Position" {
      }
      "Rotary_Point_Vector_Move" {
      }
      default {
      }
   }
}


#=============================================================
proc PB_CMD__interop_end_transition_path { } {
#=============================================================
# This command is to be called by MOM_end_of_transition_path.
#
  # Signal interop transition path has ended.
   set ::mom_sys_in_transition_path 0
}


#=============================================================
proc PB_CMD__interop_init_vars { } {
#=============================================================
# 31-Aug-2018 gsl - This command can be used to initialize some variables
#                   referenced while handling interOp GMC subops.
#
   INIT_VAR ::mom_interop_has_tool_change_container
   INIT_VAR ::mom_interop_has_tool_change_position
   INIT_VAR ::mom_interop_has_tool_change
}


#=============================================================
proc PB_CMD__interop_is_transition_path { } {
#=============================================================
# 10/31/2018 gsl - This command can be used to identify an interOp operation.
#
   if { $::mom_operation_type_enum == 900 } {
return 1
   } else {
return 0
   }
}


#=============================================================
proc PB_CMD__interop_machine_axis_move { } {
#=============================================================
OPERATOR_MSG_debug "[info level 0] interop_subop_index = $::mom_interop_subop_index"


   foreach reg { x y z a b c } {
      if { ![info exists ::mom_${reg}axis_status] } { set ::mom_${reg}axis_status -1 }
      OPERATOR_MSG_debug "::mom_${reg}axis_status = [set ::mom_${reg}axis_status]"
   }

   foreach reg { x y z a b c } {
      if { ![info exists ::mom_${reg}axis_value] } { set ::mom_${reg}axis_value -939 }
      OPERATOR_MSG_debug "::mom_${reg}axis_value = [set ::mom_${reg}axis_value]"
   }


  # Init vars to be output for machine axis moves
   UNSET_VARS ::mom_machine_coord

  # Only define variable per axis to be output - Block should set addresses to be "Optional".
  # => mom_<axis>_value carry the value w.r.t. each ref csys of individual axes.
  #    mom_pos are w.r.t the local MCS.  Depending on how the machine's datum is configured,
  #    user can determine what to output for G53 instructions.
  #
   if { $::mom_xaxis_status > 0 } {
      set ::mom_machine_coord(0) $::mom_xaxis_value
      set ::mom_machine_coord(0) $::mom_pos(0)
      set ::mom_machine_coord(0) $::mom_move_x_axis
      OPERATOR_MSG_debug "set ::mom_machine_coord(0) $::mom_machine_coord(0)"
   }
   if { $::mom_yaxis_status > 0 } {
      set ::mom_machine_coord(1) $::mom_yaxis_value
      set ::mom_machine_coord(1) $::mom_pos(1)
      set ::mom_machine_coord(1) $::mom_move_y_axis
      OPERATOR_MSG_debug "set ::mom_machine_coord(1) $::mom_machine_coord(1)"
   }
   if { $::mom_zaxis_status > 0 } {
      set ::mom_machine_coord(2) $::mom_zaxis_value
      set ::mom_machine_coord(2) $::mom_pos(2)
      set ::mom_machine_coord(2) $::mom_move_z_axis
      OPERATOR_MSG_debug "set ::mom_machine_coord(2) $::mom_machine_coord(2)"
   }

}


#=============================================================
proc PB_CMD__interop_map_machine_rotary_axes { } {
#=============================================================
  # Map 4th & 5th axis moves
  # => When leader has more than 1 character, only the 1st (0th) character is compared.
  #

  # May not need mapping below, since mom_out_angle_pos carry the angles already -
  if 0 {
   if { [PB_CMD_ask_machine_type] == "MILL" } {
      if { [string match "4_axis_*" $::mom_kin_machine_type] || [string match "5_axis_*" $::mom_kin_machine_type] } {
         set reg [string index [string tolower $::mom_kin_4th_axis_leader] 0]
         set ::mom_move_output_4th [set ::mom_move_output_${reg}]
         set ::mom_move_4th_axis   [set ::mom_move_${reg}_axis]
         set ::mom_4th_axis_status [set ::mom_${reg}axis_status]
         set ::mom_4th_axis_value  [set ::mom_${reg}axis_value]
      }
      if { [string match "5_axis_*" $::mom_kin_machine_type] } {
         set reg [string index [string tolower $::mom_kin_5th_axis_leader] 0]
         set ::mom_move_output_5th [set ::mom_move_output_${reg}]
         set ::mom_move_5th_axis   [set ::mom_move_${reg}_axis]
         set ::mom_5th_axis_status [set ::mom_${reg}axis_status]
         set ::mom_5th_axis_value  [set ::mom_${reg}axis_value]
      }
   }
  }

}


#=============================================================
proc PB_CMD__interop_start_subop_path { } {
#=============================================================
# 10/31/2018 gsl - This command can be called by MOM_start_of_subop_path handler.
#
   global mom_move_type
   global mom_move_type_name
   global mom_RTCP ;# ON/OFF

   global mom_tool_change_status

   set subop [GET_SUBOP_MOVE_NAME $::mom_move_type]

   OPERATOR_MSG_debug ">>> Start of subop >$subop< $::mom_move_type"

   incr ::mom_interop_subop_index

   switch "$subop" {
      "Rotary_Tool_Center_Point_On" {
      }
      "Rotary_Tool_Center_Point_Off" {
      }
      "Move_to_Machine_Position" {
      }
      "Rotary_Point_Vector_Move" {
      }
      default {
      }
   }

   set ::mom_user_subop_move_type_name $subop


  # Manage output of axes involved in next subop move per
  # ::mom_move_type :
  # ::mom_move_output_[a/b/c/x/y/z] : 0(Omit)/1(Specify)/2(Automatic)/3(Limit)
  #
  # When an output type is 2(Automatic), the move should be a Coordinate axis (per fixture offset)
  #
   foreach reg { x y z a b c } {
      if { ![info exists ::mom_move_output_${reg}] } { set ::mom_move_output_${reg} -1 }
      OPERATOR_MSG_debug "::mom_move_output_${reg} = [set ::mom_move_output_${reg}]"
   }

   foreach reg { x y z a b c } {
      if { ![info exists ::mom_move_${reg}_axis] } { set ::mom_move_${reg}_axis -939 }
      OPERATOR_MSG_debug "::mom_move_${reg}_axis = [set ::mom_move_${reg}_axis]"
   }

}


#=============================================================
proc PB_CMD__interop_start_transition_path { } {
#=============================================================
# This command is to be called by MOM_start_of_transition_path
#
  # Signal start of transition path, reset counter for subops within.
   set ::mom_sys_in_transition_path 1
   set ::mom_interop_subop_index -1

   if { ![info exists ::save_mom_kin_machine_type] } {
      PB_CMD_detect_tool_path_type
      set ::dpp_ge(coord_rot)  "NONE"
   }

   if { ![info exists ::mom_sys_first_tool_handled] || $::mom_sys_first_tool_handled == 0 } {

     # CSE require a tool to move -
      if { [info exists ::mom_post_in_simulation] && $::mom_post_in_simulation == "CSE" } {
         MOM_first_tool
      } else {
         MOM_do_template return_rotary_axis_to_zero
      }
   }

}


#=============================================================
proc PB_CMD__interop_transition_rapid_move { } {
#=============================================================
# This command is to be called by MOM_transition_rapid_move handler
#
OPERATOR_MSG_debug "[info level 0] interop_subop_index = $::mom_interop_subop_index"

  #------------------------------------------------
  # Skip output for start/end of transition path
   if { [string match "Rotary_Point_Vector_Move" $::mom_user_subop_move_type_name] } {

      if { $::mom_interop_subop_index == 0 } { ;# Skip start subop
         OPERATOR_MSG_debug "Skip output of start Rotary_Point_Vector_Move"
         MOM_abort_event
      }
      if { $::mom_interop_subop_index != 0 } { ;# Skip end subop
         OPERATOR_MSG_debug "Skip output of end Rotary_Point_Vector_Move"
         MOM_abort_event
      }
   }

  # When fixture offset is zero, handle it as machine axis move
   if { $::mom_fixture_offset_value <= 0 } {
      OPERATOR_MSG_debug "Handled as machine axis move"
      MOM_machine_axis_move
      MOM_abort_event
   }

}


#=============================================================
proc PB_CMD__manage_part_attributes { } {
#=============================================================
# This command allows the user to manage the MOM variables
# generated for the part attributes, in case of conflicts.
#
# ==> This command is executed automatically when present in
#     the post. DO NOT add or call it in any event or command.

# This command should only be called by MOM__part_attributes!
if { ![ CALLED_BY "MOM__part_attributes" ] } {
   return
   }
}


#=============================================================
proc PB_CMD__pattern_find_rotary_offsets { } {
#=============================================================
# Alternative way of determining the rotational offsets from the transformation matrix of current instance of a pattern.
#

# Not used for now !
return


   VMOV 12 ::mom_pattern_instance_csys_matrix CSYS_OFF

  # set mom_kin_4th_axis_leader                   "A"
  # set mom_kin_4th_axis_plane                    "YZ"
  # set mom_kin_4th_axis_type                     "Table"

   set output_factor 1.0
   if { ![string match "$::mom_output_unit" "$::mom_part_unit"] } {
      if { [string match "MM" $::mom_output_unit] } {
         set output_factor 25.4
      } else {
         set output_factor [expr 1/25.4]
      }
   }

   VMOV 5 ::mom_pos       ::mom_pos_save
   VMOV 5 ::mom_alt_pos   ::mom_alt_pos_save
   VMOV 3 ::mom_mcs_goto  ::mom_mcs_goto_save
   VMOV 3 ::mom_tool_axis ::mom_tool_axis_save

   set ::mom_mcs_goto(0)  [expr $CSYS_OFF(9)  * $output_factor]
   set ::mom_mcs_goto(1)  [expr $CSYS_OFF(10) * $output_factor]
   set ::mom_mcs_goto(2)  [expr $CSYS_OFF(11) * $output_factor]
   set ::mom_tool_axis(0) $CSYS_OFF(6)
   set ::mom_tool_axis(1) $CSYS_OFF(7)
   set ::mom_tool_axis(2) $CSYS_OFF(8)

   PB_CMD__convert_point

   set ::mom_out_angle_pos(0) [ROTSET $::mom_pos(3) $::mom_out_angle_pos(0) $::mom_kin_4th_axis_direction\
                                                    $::mom_kin_4th_axis_leader ::mom_sys_leader(fourth_axis)\
                                                    $::mom_kin_4th_axis_min_limit $::mom_kin_4th_axis_max_limit]

   set ::mom_out_angle_pos(1) [ROTSET $::mom_pos(4) $::mom_out_angle_pos(1) $::mom_kin_5th_axis_direction\
                                                    $::mom_kin_5th_axis_leader ::mom_sys_leader(fifth_axis)\
                                                    $::mom_kin_5th_axis_min_limit $::mom_kin_5th_axis_max_limit]

  # Negate rotations for table
   if { $::mom_kin_4th_axis_type == "Table" } {
      set ::mom_out_angle_pos(0) [expr -1*$::mom_out_angle_pos(0)]
   }
   if { $::mom_kin_5th_axis_type == "Table" } {
      set ::mom_out_angle_pos(1) [expr -1*$::mom_out_angle_pos(1)]
   }

   set 4th_word "${::mom_kin_4th_axis_leader}${::mom_out_angle_pos(0)}"
   set 5th_word "${::mom_kin_5th_axis_leader}${::mom_out_angle_pos(1)}"

   MOM_output_literal "G90 G00 $4th_word $5th_word"
   MOM_disable_address fourth_axis fifth_axis

}


#=============================================================
proc PB_CMD__pattern_set_csys_end { } {
#=============================================================
# This command should be called by MOM_pattern_set_csys_end to complete
# the construction of subprogram and to restore VNC configuration.
#

  # Prevent accidental call to this command
   if { ![CALLED_BY MOM_pattern_set_csys_end] } {
 return
   }

  # Complete subprogram output
   PB_CMD__subprogram_output_end

  # Restore to previous VNC configuration
   if { [info exists ::sim_mtd_initialized] && $::sim_mtd_initialized } {
      if [llength [info commands PB_VNC_pass_csys_data] ] {
         set ::mom_sim_csys_set 0
         PB_VNC_pass_csys_data
         set ::mom_sim_csys_set 1
      }
   }

}


#=============================================================
proc PB_CMD__pattern_set_csys_start { } {
#=============================================================
# This command should be called by MOM_pattern_set_csys_start event handler to initialize
# and process the transformation of coordinate system for an instance of a process pattern.
#
# Using the data resulted from the MOM core processor, this command can determine and prepare
# to output subsequent path (w.r.t. the same coordinate system) into the form of subprograms.
#

  # Prevent accidental call to this command
   if { ![CALLED_BY MOM_pattern_set_csys_start] } {
 return
   }

  # Something has gone really wrong when these vars are absent.
   if { ![info exists ::mom_process_patterning_pattern_mcs] ||\
        ![info exists ::mom_process_patterning_subroutine_program] } {
 return
   }

  # Just information -
   if { $::mom_process_pattern_index == 0 } {
     #<06-11-2019 gsl> Error with NCOPTIM obj
      if { ![info exists ::mom_process_pattern_group_name] } {
         set ::mom_process_pattern_group_name "NCOPTIM"
      }
      OPERATOR_MSG_debug "::mom_process_patterning_pattern_mcs        = $::mom_process_patterning_pattern_mcs"
      OPERATOR_MSG_debug "::mom_process_patterning_subroutine_program = $::mom_process_patterning_subroutine_program"
      OPERATOR_MSG_debug "::mom_process_pattern_group_name            = $::mom_process_pattern_group_name"
      OPERATOR_MSG_debug "::mom_process_pattern_count                 = $::mom_process_pattern_count"
      OPERATOR_MSG_debug "::mom_pattern_source_csys_matrix   : [ARR_sort_array_to_list ::mom_pattern_source_csys_matrix]"
      OPERATOR_MSG_debug "::mom_pattern_source_csys_origin   : [ARR_sort_array_to_list ::mom_pattern_source_csys_origin]"
      OPERATOR_MSG_debug "::mom_pattern_source_origin        : [ARR_sort_array_to_list ::mom_pattern_source_origin]"
   }

   OPERATOR_MSG_debug "::mom_process_pattern_index        = $::mom_process_pattern_index"
   OPERATOR_MSG_debug "::mom_pattern_csys_matrix          : [ARR_sort_array_to_list ::mom_pattern_csys_matrix]"
   OPERATOR_MSG_debug "::mom_pattern_csys_origin          : [ARR_sort_array_to_list ::mom_pattern_csys_origin]"
   OPERATOR_MSG_debug "::mom_pattern_instance_csys_matrix : [ARR_sort_array_to_list ::mom_pattern_instance_csys_matrix]"


  # Set control to output & call subroutine -
  # - Current scheme with this post enables subprogram output Only
  #   when both toggle options are checked with a Pattern Group on ONT.
  #
   if { $::mom_process_patterning_pattern_mcs == "Yes" &&\
        $::mom_process_patterning_subroutine_program == "Yes" } {
      set ::PATTERN_OUTPUT(SUB) 1
   } else {
      set ::PATTERN_OUTPUT(SUB) 0
   }


# For Dev debug -
# set ::PATTERN_OUTPUT(SUB) 0
# return


   VMOV 12 ::mom_pattern_csys_matrix  this_pattern_csys_matrix

  # Initialize some vars -
   if { ![info exists ::mom_sys_sub_program_ready] } {
      set ::mom_sys_sub_program_ready 0
   }

   if { ![info exists ::mom_fixture_offset_value] } {
      set ::mom_fixture_offset_value 0
   }


  # Configure how subprograms are numbered and incremented -
   if { $::PATTERN_OUTPUT(SUB) } {
      if { $::mom_process_pattern_index == 0 } {

         set ::mom_sys_sub_program_ready 0

        # Initialize subprogram number -
         if { ![info exists ::mom_sys_subprogram_number] } {
            set ::mom_sys_subprogram_number 1000
         } else {
            incr ::mom_sys_subprogram_number 100
         }

      } else {

        # Increment fixture offset number for subsequent subprogram calls.
         incr ::mom_fixture_offset_value
      }
   }


  #-------------------------------------
  # No need to process the 0th instance
  #
   if { $::mom_process_pattern_index == 0 } {
     # The 1st subprogram call will use current fixture offset number.
return
   }


  #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  # @ Very first oper of the 0th instance, mom_csys_matrix is not yet set!
  #
   if ![info exists ::mom_csys_matrix] {
return
   }



 set instance_index [expr $::mom_process_pattern_index + 1]
 OPERATOR_MSG "PATTERN : $::mom_process_pattern_group_name #$instance_index INSTANCE OF $::mom_process_pattern_count"


  # Find the rotational & linear offsets from the transformation matrix of current instance of a pattern,
  # and extract data to output NC instructions of coordinate system changes before calling a subprogram.
  #
   VMOV 12 ::mom_pattern_instance_csys_matrix CSYS_OFF

   CSYS_ask_rotary_angles ANG_A ANG_B ANG_C CSYS_OFF

   if { ![EQ_is_zero $ANG_A] || ![EQ_is_zero $ANG_B] || ![EQ_is_zero $ANG_C] } {

     # Negate rotations for table
      set ANG_A [format "%.4f" [expr -1*$ANG_A]]
      set ANG_B [format "%.4f" [expr -1*$ANG_B]]
      set ANG_C [format "%.4f" [expr -1*$ANG_C]]

      MOM_output_literal "G90 G00 A$ANG_A B$ANG_B C$ANG_C"
      MOM_disable_address fourth_axis fifth_axis

      MOM_output_literal "G69"
      if { ![EQ_is_zero $ANG_A] } {
         MOM_output_literal "G68 I1 R$ANG_A"
      }
      if { ![EQ_is_zero $ANG_B] } {
         MOM_output_literal "G68 J1 R$ANG_B"
      }
      if { ![EQ_is_zero $ANG_C] } {
         MOM_output_literal "G68 K1 R$ANG_C"
      }
   }


   set output_factor 1.0
   if { ![string match "$::mom_output_unit" "$::mom_part_unit"] } {
      if { [string match "MM" $::mom_output_unit] } {
         set output_factor 25.4
      } else {
         set output_factor [expr 1/25.4]
      }
   }


   if { [EQ_is_zero $ANG_A] && [EQ_is_zero $ANG_B] && [EQ_is_zero $ANG_C] } {
      set CSYS_OFF(9)  0.0
      set CSYS_OFF(10) 0.0
      set CSYS_OFF(11) 0.0
   } else {
      if { ![EQ_is_zero $ANG_A] } {
         set CSYS_OFF(9) 0.0
      }
      if { ![EQ_is_zero $ANG_B] } {
         set CSYS_OFF(10) 0.0
      }
      if { ![EQ_is_zero $ANG_C] } {
         set CSYS_OFF(11) 0.0
      }
   }


   set this_pattern_csys_matrix(9)   [expr $this_pattern_csys_matrix(9)  - $CSYS_OFF(9)]
   set this_pattern_csys_matrix(10)  [expr $this_pattern_csys_matrix(10) - $CSYS_OFF(10)]
   set this_pattern_csys_matrix(11)  [expr $this_pattern_csys_matrix(11) - $CSYS_OFF(11)]


  # Output a local shift instruction
   MOM_output_literal "G52 X0.0 Y0.0 Z 0.0"
   PB_CMD_fixture_offset

   if { [EQ_is_zero $ANG_A] && [EQ_is_zero $ANG_B] && [EQ_is_zero $ANG_C] } {

      set X_shift [format "%.4f" [expr $this_pattern_csys_matrix(9)  * $output_factor - $::mom_csys_origin(0)]]
      set Y_shift [format "%.4f" [expr $this_pattern_csys_matrix(10) * $output_factor - $::mom_csys_origin(1)]]
      set Z_shift [format "%.4f" [expr $this_pattern_csys_matrix(11) * $output_factor - $::mom_csys_origin(2)]]

      MOM_output_literal "G52 X$X_shift Y$Y_shift Z$Z_shift"
   }


   # Call subprogram for current instance
   if { $::PATTERN_OUTPUT(SUB) } {
      MOM_output_literal "M98 P$::mom_sys_subprogram_number"
      MOM_output_text " "
   }


if 0 {
PAUSE "A = $ANG_A, B = $ANG_B, C = $ANG_C\n\n\
       pattern_csys_matrix = [ARR_sort_array_to_list ::mom_pattern_csys_matrix]\n\n\
       csys_matrix         = [ARR_sort_array_to_list ::mom_csys_matrix] [ARR_sort_array_to_list ::mom_csys_origin]\n\n\
       msys_matrix         = [ARR_sort_array_to_list ::mom_msys_matrix] [ARR_sort_array_to_list ::mom_msys_origin]"
}


  # Pass the CSYS information that will be used to
  # set the ZCS coordinate system for simulation.
  #
   if { ![EQ_is_zero $ANG_A] || ![EQ_is_zero $ANG_B] || ![EQ_is_zero $ANG_C] } {

      if { [info exists ::sim_mtd_initialized] && $::sim_mtd_initialized } {
         if [llength [info commands PB_VNC_pass_csys_data] ] {
            VMOV 12 ::mom_csys_matrix saved_csys_matrix
           # VMOV 9  ::mom_csys_matrix this_pattern_csys_matrix
            VMOV 9  ::mom_pattern_source_csys_matrix this_pattern_csys_matrix
            VMOV 12  this_pattern_csys_matrix ::mom_csys_matrix

            set ::mom_sim_csys_set 0
            PB_VNC_pass_csys_data
            set ::mom_sim_csys_set 1

            VMOV 12 saved_csys_matrix ::mom_csys_matrix
         }
      }
   }

}


#=============================================================
proc PB_CMD__subprogram_output_dump { } {
#=============================================================
# This command should be called by MOM_end_of_program
# under the condition when subprogram output is enabled.
#

   if { [info exists ::PATTERN_OUTPUT(SUB)] && $::PATTERN_OUTPUT(SUB) } {

      if { [info exists  ::mom_sys_subprogram_output_file] &&\
           [file exists $::mom_sys_subprogram_output_file] } {

         if { [file size $::mom_sys_subprogram_output_file] } {

           #---------------------------------------------------------
           # Combine subprograms with contents of existing N/C file -
           #---------------------------------------------------------
            MOM_close_output_file $::ptp_file_name

            if [file exists __tmp_output_file] {
               file delete -force __tmp_output_file
            }
            file rename $::ptp_file_name __tmp_output_file

           # Open a fresh PTP file -
            MOM_open_output_file $::ptp_file_name

           # Construct final program -
            MOM_suppress Once N
            MOM_do_template rewind_stop_code

           # Append subprograms -
            set src [open "$::mom_sys_subprogram_output_file" RDONLY]
            while { [eof $src] == 0 } {
               MOM_output_text [gets $src]
            }
            close $src

           # Append contents of PTP file -
            set src [open __tmp_output_file RDONLY]
            while { [eof $src] == 0 } {
               set line [gets $src]
               if { $line != "%" } {
                  MOM_output_text $line
               }
            }
            close $src
            file delete __tmp_output_file

            MOM_enable_address N
         }

         file delete "$::mom_sys_subprogram_output_file"
      }
   }

}


#=============================================================
proc PB_CMD__subprogram_output_end { } {
#=============================================================
# This command is called by PB_CMD__pattern_set_csys_end, currently;
# it will complete the construction of a subprogram and make the initial call.
#

   if { [info exists ::PATTERN_OUTPUT(SUB)] && $::PATTERN_OUTPUT(SUB) &&\
        $::mom_process_pattern_index == 0 } {

     # End current subprogram
      MOM_output_text "M99"
      MOM_output_text " "

      MOM_close_output_file $::mom_sys_subprogram_output_file

     # Resume main program output
      MOM_open_output_file  $::ptp_file_name

     # Make the first call (0th instance)
      MOM_output_text " "

 set instance_index [expr $::mom_process_pattern_index + 1]
 OPERATOR_MSG "PATTERN : $::mom_process_pattern_group_name #$instance_index INSTANCE OF $::mom_process_pattern_count"

      PB_CMD_fixture_offset

      MOM_output_literal "M98 P$::mom_sys_subprogram_number"
      MOM_output_text " "

      set ::PATTERN_OUTPUT(SUB) 0
   }

}


#=============================================================
proc PB_CMD__subprogram_output_start { } {
#=============================================================
# This command should be called by MOM_initial_move & MOM_first_move
# under the condition when subprogram output is enabled.
#

  # Start building of a subprogram, close it off @ MOM_pattern_set_csys_end
  #
   if { [info exists ::PATTERN_OUTPUT(SUB)] && $::PATTERN_OUTPUT(SUB) &&\
        $::mom_process_pattern_index == 0 } {

      if { ![info exists ::mom_sys_sub_program_ready] } {
         set ::mom_sys_sub_program_ready 0
      }

     # Suspend current output file
      MOM_close_output_file $::ptp_file_name

     # Prepare output file to collect subprograms
      if { ![info exists  ::mom_sys_subprogram_output_file] ||\
           ![file exists $::mom_sys_subprogram_output_file] } {

         set out_file "${::mom_output_file_directory}${::mom_logname}__tmp_output_[clock clicks].spf"

         set ::mom_sys_subprogram_output_file "${out_file}"
      }

      MOM_open_output_file "${::mom_sys_subprogram_output_file}"

     # Output subprogram name
      MOM_disable_address N
      if { $::mom_sys_sub_program_ready == 0 } {
         MOM_output_text "O${::mom_sys_subprogram_number}"
         PB_CMD_oper_info
      }

      set ::mom_sys_sub_program_ready 1
   }

}


#=============================================================
proc PB_CMD__suppress_probe_bore_clearance_retract { } {
#=============================================================
# - For Probing Operation -
#
# By default, NX/Post generates protected moves to enter and retract from
# the bore feature of a probing cycle when the radial clearance is zero.
#
# ==> This command can be attached to the "Start of Program" event
#     to suppress the retract motion of the protected move.
#
# 19-Mar-2019 gsl - New
#
   set ::mom_sys_suppress_probe_bore_clearance_retract 1
}


#=============================================================
proc PB_CMD__validate_4_axis_motion { } {
#=============================================================
# Validate legitimate motion for 4-axis post -
#
# => "mom_spindle_axis" accounts for the direction change resulted from the angled-head attachment added to the spindle.
#
# For a 4-axis Table - The spindle axis (Vs) and tool axis (Vt) should be either co-linear '||'
#                      BOTH on the plane of rotation (Vp).
# For a 4-axis Head  - The spindle axis (Vs) MUST lie ON the plane of rotation (Vp) '&&'
#                      identical to the tool axis (Vt).
#
# - The max/min of the rotary axis will further constraint the reachability.
# - Vectors' DOT product will be 0 or +/-1. (Vt.Vp => 0 || +/-1)
#
#-------------------------------------------------------------
# 04-29-2015 gsl - Carved out of LOCK_AXIS_MOTION to be called in MOM_before_motion

if { [string match "4_axis_table" $::mom_kin_machine_type] } {
   if { !([EQ_is_equal [expr abs( [VEC3_dot ::mom_sys_spindle_axis ::mom_tool_axis] )] 1.0] || \
         ([EQ_is_equal [VEC3_dot ::mom_sys_spindle_axis ::mom_kin_4th_axis_vector] 0.0]     && \
          [EQ_is_equal [VEC3_dot ::mom_tool_axis        ::mom_kin_4th_axis_vector] 0.0])) } {
      CATCH_WARNING "Illegal motion for 4-axis table machine"

      MOM_abort_event

      return 0
      }
   }

if { [string match "4_axis_head" $::mom_kin_machine_type] } {
   if { !([EQ_is_equal [VEC3_dot ::mom_sys_spindle_axis ::mom_kin_4th_axis_vector] 0.0] && \
          [EQ_is_equal [VEC3_dot ::mom_sys_spindle_axis ::mom_tool_axis] 1.0]) } {
      CATCH_WARNING "Illegal motion for 4-axis head machine"

      MOM_abort_event

      return 0
      }
   }

return 1
}


#=============================================================
proc PB_CMD__validate_motion { } {
#=============================================================
# Validate legitimate motion outputs of different post configurations -
# ==> Do not rename this command!
#
# For a 4-axis Table - The spindle axis (Vs) and tool axis (Vt) should be either co-linear or (||)
#                      BOTH on the plane of rotation (Vp).
# For a 4-axis Head  - The spindle axis (Vs) should be identical to the tool axis (Vt) and (&&)
#                      must lie ON the plane of rotation (Vp).
#
# - "mom_spindle_axis" has accounted for the direction change resulted from
#   the angled-head attachment added to the spindle.
# - The max/min of the rotary axis will further constraint the reachability.
# - Vectors' DOT product will be 0 or +/-1. (Vt.Vp => 0 || +/-1)
#
# ==> This command can be enhanced to validate outputs of other post configurations.
#
#   Return: 1 = Motion OK
#           0 = Motion Bad
#-------------------------------------------------------------

# return 1

# "mom_spindle_axis" would include transformation of head attachment.
if [info exists ::mom_spindle_axis] {
   VMOV 3 ::mom_spindle_axis ::mom_sys_spindle_axis

   } else {
          VMOV 3 ::mom_kin_spindle_axis ::mom_sys_spindle_axis
          }

if { [string match "4_axis_table" $::mom_kin_machine_type] } {
   if { !([EQ_is_equal [expr abs([VEC3_dot ::mom_sys_spindle_axis ::mom_tool_axis])] 1.0] || \
         ([EQ_is_equal [VEC3_dot ::mom_sys_spindle_axis ::mom_kin_4th_axis_vector] 0.0]   && \
          [EQ_is_equal [VEC3_dot ::mom_tool_axis        ::mom_kin_4th_axis_vector] 0.0])) } {
      CATCH_WARNING "Illegal motion for 4-axis table machine"

      return 0
      }
   }

if { [string match "4_axis_head" $::mom_kin_machine_type] } {
   if { !([EQ_is_equal [VEC3_dot ::mom_sys_spindle_axis ::mom_tool_axis] 1.0] && \
          [EQ_is_equal [VEC3_dot ::mom_sys_spindle_axis ::mom_kin_4th_axis_vector] 0.0]) } {
      CATCH_WARNING "Illegal motion for 4-axis head machine"

      return 0
      }
   }

return 1
}


#=============================================================
proc PB_CMD_abort_event { } {
#=============================================================
# This command can be called to abort an event based on the
# flag being set by other handler under certain conditions,
# such as an invalid tool axis vector.
#
# Users can set the global variable mom_sys_abort_next_event to
# different severity levels throughout the post and designate
# how to handle different conditions in this command.
#
# - Rapid, linear, circular and cycle move events have this trigger
#   built in by default in PB6.0.
#
# 09-17-2015 szl - Output a warning message in NC output while postprocessor
#                  cannot calculate the valid rotary position.

global mom_warning_info
global mom_sys_warning_output
global mom_sys_abort_next_event
global mom_sys_warning_output_option

if { [info exists mom_sys_abort_next_event] } {
   switch $mom_sys_abort_next_event {
                                    1 {
                                      unset mom_sys_abort_next_event

                                      if { ![string compare "WARNING: unable to determine valid rotary positions" $mom_warning_info] } {
                                         set save_mom_sys_warning_putput $mom_sys_warning_output
                                         set save_mom_sys_warning_output_option $mom_sys_warning_output_option

                                         set mom_sys_warning_output "ON"
                                         set mom_sys_warning_output_option "LIST"

                                         MOM_catch_warning

                                         set mom_sys_warning_output $save_mom_sys_warning_putput
                                         set mom_sys_warning_output_option $save_mom_sys_warning_output_option
                                         }
                                      }
                                    2 {
                                      unset mom_sys_abort_next_event

                                      CATCH_WARNING "Event aborted!"

                                      MOM_abort_event
                                      }
                              default {
                                      unset mom_sys_abort_next_event

                                      CATCH_WARNING "Event warned!"
                                      }
                                    }
   }
}


#=============================================================
proc PB_CMD_ask_machine_type { } {
#=============================================================
# Utility to return machine type per mom_kin_machine_type

global mom_kin_machine_type

if { [string match "*wedm*" $mom_kin_machine_type] } {
   return WEDM

   } elseif { [string match "*axis*" $mom_kin_machine_type] } {
            return MILL

            } elseif { [string match "*lathe*" $mom_kin_machine_type] } {
                     return TURN

                     } else {
                            return $mom_kin_machine_type
                            }
}


#=============================================================
proc PB_CMD_before_motion { } {
#=============================================================
global mom_pos
global mom_prev_pos
global mom_mcs_goto
global mom_arc_center
global mom_prev_mcs_goto
global mom_pos_arc_center

global dpp_ge
global mom_user_curr_pos
global mom_user_prev_pos

# Preserve mom_pos & mom_prev_pos
VMOV 3 mom_pos      mom_user_curr_pos
VMOV 3 mom_prev_pos mom_user_prev_pos

if { $dpp_ge(sys_output_coord_mode) == "TCP_FIX_TABLE" && $dpp_ge(toolpath_axis_num) == "5" } {
   VMOV 3 mom_mcs_goto mom_pos
   VMOV 3 mom_prev_mcs_goto mom_prev_pos
   VMOV 3 mom_arc_center mom_pos_arc_center
   }
}


#=============================================================
proc PB_CMD_before_output { } {
#=============================================================
# This command allows users to massage the NC data (mom_o_buffer) before
# it finally gets output.  If present in the post, this command gets executed
# by MOM_before_output automatically.
#
# - DO NOT overload MOM_before_output! All customization should be done here!
# - DO NOT call any MOM output commands in this command, it will become cyclicle!
# - No need to attach this command to any event marker.

global mom_o_buffer
global mom_sys_leader
global mom_sys_control_in
global mom_sys_control_out
}


#=============================================================
proc PB_CMD_cal_feedrate_by_pitch_and_ss { } {
#=============================================================
# Calculate feedrate by thread pitch and spindle speed.
#
# 2014-03-20 levi - Initial version.
# 2015-08-21 szl  - Enhance the warning message when users set wrong pitch and wrong spindle speed,fix PR7463004.

global feed
global feed_mode
global mom_tool_name
global mom_tool_pitch
global mom_spindle_rpm
global mom_spindle_speed
global mom_feed_cut_unit
global mom_operation_name
global mom_cycle_feed_rate
global mom_cycle_thread_pitch
global mom_cycle_feed_rate_mode

# Calculate the pitch, get it from model first, if can't get from model, use the pitch of tool.
if { [info exists mom_tool_pitch] } {
   if { [info exists mom_cycle_thread_pitch] } {
      set pitch $mom_cycle_thread_pitch

      } else {
             set pitch $mom_tool_pitch
             }

   } else {
          MOM_display_message "$mom_operation_name: No pitch defined on the tool. Please use Tap tool. \n Post Processing will be aborted." "Postprocessor error message" "E"
          MOM_abort "*** User Abort Post Processing *** "
          }

# Calculate the F parameter of cycle, if the feedrate mode is MMPR or IPR, use pitch as feedrate,
# if the feedrate mode is MMPM or IPM, calculate it by $pitch*$mom_spindle_speed. Don't use the feedrate
# value set in NX directly.
if { ![info exists mom_spindle_speed] || [EQ_is_zero $mom_spindle_speed] } {
   MOM_display_message "$mom_operation_name : spindle speed is 0. Post Processing will be aborted." "Postprocessor error message" "E"
   MOM_abort "*** User Abort Post Processing *** "
   }

if { [string match "*PR" $feed_mode] } {
   set feed $pitch

   } else {
          set feed [expr $pitch*$mom_spindle_rpm]
          }
}


#=============================================================
proc PB_CMD_cancel_suppress_force_once_per_event { } {
#=============================================================
# This command can be called to cancel the effect of
# "MOM_force Once" & "MOM_suppress Once" for each event.
#
# => It's to keep the effect of force & suppress once within
#    the scope of the event that issues the commands and
#    eliminate the unexpected residual effect of such commands
#    that may have been issued by other events.
#
# PB v11.02 -
#
   MOM_cancel_suppress_force_once_per_event
}


#=============================================================
proc PB_CMD_check_output_mode_validity { } {
#=============================================================
# Check the validity of the output mode
# Leave this command here to do customization


}


#=============================================================
proc PB_CMD_check_plane_change_for_swiveling { } {
#=============================================================
# This command is to deal with the condition that when cutting material, tool
# axis is fixed, when non-cutting, tool axis is variable. It's swiveling mode, output G68.2.
#
# 06-04-2013 levi - Initial version
# 10-18-2013 levi - When tool axis change back to straight in 3+2 operation, recalculate mom_pos using the original kinematics.

global mom_pos
global mom_result
global mom_mcs_goto
global mom_prev_pos
global mom_tool_axis
global mom_cycle_feed_to
global mom_out_angle_pos
global mom_current_motion
global mom_cycle_rapid_to
global mom_operation_type
global mom_kin_machine_type
global mom_cycle_retract_to
global mom_cycle_feed_to_pos
global mom_prev_out_angle_pos
global mom_cycle_rapid_to_pos
global mom_cycle_retract_to_pos
global mom_cutcom_adjust_register

global dpp_ge
global first_hole_flag
global first_rapid_move_status
global save_mom_kin_machine_type

if { [string match "*3_axis*" $save_mom_kin_machine_type] || [string match "*4_axis*" $save_mom_kin_machine_type] } {
   return
   }

if { $dpp_ge(toolpath_axis_num) == "3" && $dpp_ge(sys_coord_rotation_output_type) == "SWIVELING" } {
   if { [string compare "LOCAL" $dpp_ge(coord_rot)] } {
      if {![info exists mom_prev_out_angle_pos]} {
         set mom_prev_out_angle_pos(0) 0.0
         set mom_prev_out_angle_pos(1) 0.0
         }

      if { ![EQ_is_equal $mom_out_angle_pos(0) $mom_prev_out_angle_pos(0)] || \
           ![EQ_is_equal $mom_out_angle_pos(1) $mom_prev_out_angle_pos(1)] } {
         set dpp_ge(coord_rot) [DPP_GE_COOR_ROT "ZXZ" angle offset pos]

         if { [string compare "NONE" $dpp_ge(coord_rot)] } {
            for { set i 0 } { $i < 3} { incr i } {
                set dpp_ge(coord_offset,$i) $offset($i)
                set dpp_ge(coord_rot_angle,$i) $angle($i)
                }

            if { ![EQ_is_equal $dpp_ge(coord_rot_angle,0) $dpp_ge(prev_coord_rot_angle,0)] ||\
                 ![EQ_is_equal $dpp_ge(coord_rot_angle,1) $dpp_ge(prev_coord_rot_angle,1)] ||\
                 ![EQ_is_equal $dpp_ge(coord_rot_angle,2) $dpp_ge(prev_coord_rot_angle,2)] } {

               VMOV 3 pos mom_pos

               MOM_reload_variable -a mom_pos

               MOM_force once G_dwo
               MOM_do_template cancel_dynamic_work_offset

               MOM_force once G_offset G_motion Z
               MOM_do_template return_home_z

               MOM_force once M_clamp_5th Text
               MOM_do_template fifth_axis_unclamp
               MOM_force once M_clamp_4th Text
               MOM_do_template fourth_axis_unclamp

               MOM_force once G_motion fourth_axis fifth_axis
               MOM_do_template initial_move_rotation

               MOM_force once G_dwo
               MOM_do_template activate_dynamic_work_offset

               MOM_force once M_clamp_5th Text
               MOM_do_template fifth_axis_clamp
               MOM_force once M_clamp_4th Text
               MOM_do_template fourth_axis_clamp

               MOM_force once G_offset

               if { [info exists first_hole_flag] }         { unset first_hole_flag }
               if { [info exists first_rapid_move_status] } { unset first_rapid_move_status }

               for { set i 0 } { $i < 3 } { incr i } {
                   set dpp_ge(prev_coord_rot_angle,$i) $dpp_ge(coord_rot_angle,$i)
                   }
               }
            } else {
                   if { [string match "Drilling" $mom_operation_type] || [string match "Point to Point" $mom_operation_type] || [string match "Hole Making" $mom_operation_type] || [string match "Cylinder Milling" $mom_operation_type] } {
                      if { (![EQ_is_zero [expr $mom_prev_out_angle_pos(0) + $mom_prev_out_angle_pos(1)]]) && ![string match "initial_move" $mom_current_motion] } {
                         MOM_force once G_dwo
                         MOM_do_template cancel_dynamic_work_offset

                         MOM_force once G_offset G_motion Z
                         MOM_do_template return_home_z

                         MOM_force once M_clamp_5th Text
                         MOM_do_template fifth_axis_unclamp
                         MOM_force once M_clamp_4th Text
                         MOM_do_template fourth_axis_unclamp

                         MOM_force once G_motion fourth_axis fifth_axis
                         MOM_do_template initial_move_rotation

                         MOM_force once M_clamp_5th Text
                         MOM_do_template fifth_axis_clamp
                         MOM_force once M_clamp_4th Text
                         MOM_do_template fourth_axis_clamp

                         MOM_force once G_offset

                         if { [info exists first_hole_flag] }         { unset first_hole_flag }
                         if { [info exists first_rapid_move_status] } { unset first_rapid_move_status }
                         }
                      }

                   set dpp_ge(prev_coord_rot_angle,0) 0
                   set dpp_ge(prev_coord_rot_angle,1) 0
                   set dpp_ge(prev_coord_rot_angle,2) 0

                   # if it's not auto3d condition, restore the kinematics and recalculate mom_pos
                   DPP_GE_RESTORE_KINEMATICS

                   if { "1" == [MOM_convert_point mom_mcs_goto mom_tool_axis] } {
                      set i 0

                      foreach value $mom_result {
                              set mom_pos($i) $value

                              incr i
                              }
                      }

                   MOM_reload_variable -a mom_pos
                   }
         }
      }
   }
}


#=============================================================
proc PB_CMD_check_plane_change_for_wcs_rotation { } {
#=============================================================
# This command is to deal with the condition that when cutting material, tool
# axis is fixed, when non-cutting, tool axis is variable. It's wcs rotation mode, output G68.
#
# 06-04-2013 levi - Initial version
# 10-18-2013 levi - When tool axis change back to straight in 3+2 operation, recalculate mom_pos using the original kinematics.

  global dpp_ge
  global mom_out_angle_pos mom_prev_out_angle_pos mom_cycle_rapid_to_pos mom_cycle_feed_to_pos mom_cycle_retract_to_pos mom_pos
  global mom_cycle_rapid_to mom_cycle_retract_to mom_cycle_feed_to
  global mom_cutcom_adjust_register
  global mom_mcs_goto
  global save_mom_kin_machine_type
  global mom_tool_axis
  global mom_result
  global mom_prev_pos

  if { $save_mom_kin_machine_type == "5_axis_dual_table" || [string match "*4_axis*" $save_mom_kin_machine_type] ||\
       [string match "*3_axis*" $save_mom_kin_machine_type] || $dpp_ge(coord_rot) == "LOCAL" } {
     return
  }

  if { $dpp_ge(toolpath_axis_num) == "3" && $dpp_ge(sys_coord_rotation_output_type) == "WCS_ROTATION" } {
     if { ![info exists mom_prev_out_angle_pos] } {
        set mom_prev_out_angle_pos(0) 0.0
        set mom_prev_out_angle_pos(1) 0.0
     }

     if { ![EQ_is_equal $mom_out_angle_pos(0) $mom_prev_out_angle_pos(0)] || \
          ![EQ_is_equal $mom_out_angle_pos(1) $mom_prev_out_angle_pos(1)] } {
        set dpp_ge(coord_rot) [DPP_GE_COOR_ROT_WCS_ROTATION  g68_first_vec g68_second_vec coord_rot_angle coord_offset pos]

        if { [info exists mom_cutcom_adjust_register] } {
           MOM_output_literal "G40"
           MOM_force once G_cutcom D
        }
        MOM_output_literal "G49"
        MOM_output_literal "G69"
        MOM_force once G_adjust H X Y Z
        if { [string compare "NONE" $dpp_ge(coord_rot)] } {
           for { set i 0 } { $i < 3 } { incr i } {
              set dpp_ge(g68_first_vec,$i)        $g68_first_vec($i)
              set dpp_ge(g68_second_vec,$i)       $g68_second_vec($i)
              set dpp_ge(coord_offset,$i)         $coord_offset($i)
              set dpp_ge(coord_offset2,$i)        0
              set dpp_ge(coord_rot_angle,$i)      $coord_rot_angle($i)
              set dpp_ge(prev_g68_first_vec,$i)   $dpp_ge(g68_first_vec,$i)
              set dpp_ge(prev_g68_second_vec,$i)  $dpp_ge(g68_second_vec,$i)
              set dpp_ge(prev_coord_offset,$i)    $dpp_ge(coord_offset,$i)
              set dpp_ge(prev_coord_rot_angle,$i) $dpp_ge(coord_rot_angle,$i)
              set mom_pos($i)                     $pos($i)
           }

           MOM_reload_variable -a mom_pos

           # Output rotary angle
           MOM_do_template initial_move_rotation

           # Adjust the output for G68 if one or two of the angles are 0, under this condition just output G68 once or less,
           # otherwise should output G68 twice
           if { [EQ_is_equal $coord_rot_angle(0) 0] } {
              if { ![EQ_is_equal $coord_offset(0) 0] || ![EQ_is_equal $coord_offset(1) 0] || ![EQ_is_equal $coord_offset(2) 0] } {
                 VMOV 3 g68_second_vec g68_first_vec
                 set dpp_ge(g68_first_vec,0) $g68_first_vec(0)
                 set dpp_ge(g68_first_vec,1) $g68_first_vec(1)
                 set dpp_ge(g68_first_vec,2) $g68_first_vec(2)
                 set coord_rot_angle(0) $coord_rot_angle(1)
                 MOM_force once rotate_X rotate_Y rotate_Z rotate_i rotate_j rotate_k rotate_r
                 MOM_do_template g68_first_coord_rot
              } elseif { ![EQ_is_equal $coord_rot_angle(1) 0] } {
                 MOM_force once rotate_X rotate_Y rotate_Z rotate_i rotate_j rotate_k rotate_r
                 MOM_do_template g68_second_coord_rot
              }
           } elseif { [EQ_is_equal $coord_rot_angle(1) 0] } {
              MOM_force once rotate_X rotate_Y rotate_Z rotate_i rotate_j rotate_k rotate_r
              MOM_do_template g68_first_coord_rot
           } else {
              MOM_force once rotate_X rotate_Y rotate_Z rotate_i rotate_j rotate_k rotate_r
              MOM_do_template g68_first_coord_rot
              MOM_force once rotate_X rotate_Y rotate_Z rotate_i rotate_j rotate_k rotate_r
              MOM_do_template g68_second_coord_rot
           }
        } else {
           for { set i 0 } { $i < 3 } { incr i } {
              set $dpp_ge(prev_g68_first_vec,$i) 0
              set $dpp_ge(prev_g68_second_vec,$i) 0
              set $dpp_ge(prev_coord_offset,$i) 0
              set $dpp_ge(prev_coord_rot_angle,$i) 0
           }


          # If it's not auto3d condition, restore the kinematics and recalculate mom_pos
           DPP_GE_RESTORE_KINEMATICS
           PB_CMD__convert_point

           MOM_reload_variable -a mom_pos
           MOM_do_template initial_move_rotation
        }
     }
  }

}


#=============================================================
proc PB_CMD_clamp_fifth_axis { } {
#=============================================================
#  This procedure is used by auto clamping to output the code
#  needed to clamp the fifth axis.
#
#  Do NOT add this block to events or markers.  It is a static
#  block and it is not intended to be added to events.  Do NOT
#  change the name of this custom command.

MOM_do_template fifth_axis_clamp
}


#=============================================================
proc PB_CMD_clamp_fourth_axis { } {
#=============================================================
#  This procedure is used by auto clamping to output the code
#  needed to clamp the fourth axis.
#
#  Do NOT add this block to events or markers.  It is a static
#  block and it is not intended to be added to events.  Do NOT
#  change the name of this custom command.

MOM_do_template fourth_axis_clamp
}


#=============================================================
proc PB_CMD_combine_rotary_check { } {
#=============================================================
#  Combine consecutive rotary moves
#
#  These custom commands will allow you to combine consecutive
#  rotary moves into a single move when there is no change in
#  X, Y and Z.
#
#  The combining of blocks will terminate when the rotary axis
#  being combined reverses or the total number of degrees of
#  the combined rotary move would have exceeded 180 degrees.
#
#  The current linear move will be suppressed if the current and the
#  next motion are either CUT or FIRSTCUT and both are linear
#  moves.
#
#  This function will only work with NX3 or later.
#  Add the follow line (without the #) to the custom command
#  PB_CMD_before_motion.
#PB_CMD_combine_rotary_check
#
#  Select the custom command PB_CMD_combine_rotary_output from the
#  pulldown in the Linear Move event and drag it in into the start
#  of the Linear Move.  The Linear Move event is located at
#  Program & Tool Path \ Program \ Tool Path \ Motion \.
#
#  Select the custom command PB_CMD_combine_rotary_init from the
#  pulldown in the Start_of_Program event and drag it in into the
#  Start of Program event marker.  The Start of program event is
#  located at Program & Tool Path \ Program \ Program Start Sequence.
#
#  The following variables can be changed to relect the number of
#  decimal places that will be output for linear and rotary words.

set linear_decimals 4
set rotary_decimals 3

global mom_pos
global mom_nxt_pos
global mom_prev_pos
global mom_motion_type
global mom_nxt_motion_type

global dpp_ge
global last_4th_dir
global last_5th_dir
global prev_4th_output
global prev_5th_output
global last_4th_output
global last_5th_output

set dpp_ge(skip_move) "NO"

if { ![info exists prev_4th_output] } { set prev_4th_output $mom_pos(3) }
if { ![info exists prev_5th_output] } { set prev_5th_output $mom_pos(4) }

set P4 [format "%.${rotary_decimals}f" $prev_4th_output]
set P5 [format "%.${rotary_decimals}f" $prev_5th_output]

set prev_4th_output $mom_pos(3)
set prev_5th_output $mom_pos(4)

if { ![info exists last_4th_output] } { set last_4th_output $P4 }
if { ![info exists last_5th_output] } { set last_5th_output $P5 }

if { ![info exists last_4th_dir] } { set last_4th_dir 0 }
if { ![info exists last_5th_dir] } { set last_5th_dir 0 }

if { [info exists mom_nxt_pos] && [info exists mom_nxt_motion_type] } {
   set PX [format "%.${linear_decimals}f" $mom_prev_pos(0)]
   set PY [format "%.${linear_decimals}f" $mom_prev_pos(1)]
   set PZ [format "%.${linear_decimals}f" $mom_prev_pos(2)]

   set NX [format "%.${linear_decimals}f" $mom_nxt_pos(0)]
   set NY [format "%.${linear_decimals}f" $mom_nxt_pos(1)]
   set NZ [format "%.${linear_decimals}f" $mom_nxt_pos(2)]

   set N4 [format "%.${rotary_decimals}f" $mom_nxt_pos(3)]
   set N5 [format "%.${rotary_decimals}f" $mom_nxt_pos(4)]

   set X [format "%.${linear_decimals}f" $mom_pos(0)]
   set Y [format "%.${linear_decimals}f" $mom_pos(1)]
   set Z [format "%.${linear_decimals}f" $mom_pos(2)]

   set R4 [format "%.${rotary_decimals}f" $mom_pos(3)]
   set R5 [format "%.${rotary_decimals}f" $mom_pos(4)]

   set D4 [expr $R4-$P4]

   if [EQ_is_equal $D4 0.0] {
      set cur_4th_dir 0

      } elseif { ($D4 > -180.0 && $D4 < 0.0) || ($D4 > 180.0) } {
               set cur_4th_dir -1

               } else {
                      set cur_4th_dir 1
                      }

   set T4 [expr $N4-$last_4th_output]

   if [EQ_is_equal $T4 0.0] {
      set tot_4th_dir 0

      } elseif { ($T4 > -180.0 && $T4 < 0.0) || ($T4 > 180.0) } {
               set tot_4th_dir -1

               } else {
                      set tot_4th_dir 1
                      }

   if { [expr $cur_4th_dir*$last_4th_dir] < -.5 || [expr $cur_4th_dir*$tot_4th_dir] < -.5 } {
      set switch_dir_4th "YES"

      } else {
             set switch_dir_4th "NO"
             }

   set D5 [expr $R5-$P5]

   if [EQ_is_equal $D5 0.0] {
      set cur_5th_dir 0

      } elseif { ($D5 > -180.0 && $D5 < 0.0) || ($D5 > 180.0) } {
               set cur_5th_dir -1

               } else {
                      set cur_5th_dir 1
                      }

   set T5 [expr $N5-$last_5th_output]

   if [EQ_is_equal $T5 0.0] {
      set tot_5th_dir 0

      } elseif { ($T5 > -180.0 && $T5 < 0.0) || ($T5 > 180.0) } {
               set tot_5th_dir -1

               } else {
                      set tot_5th_dir 1
                      }

   if { [expr $cur_5th_dir*$last_5th_dir] < -.5 || [expr $cur_5th_dir*$tot_5th_dir] < -.5 } {
      set switch_dir_5th "YES"

      } else {
             set switch_dir_5th "NO"
             }

   if { ($mom_motion_type == "CUT" && $mom_nxt_motion_type == "CUT") || ($mom_motion_type == "FIRSTCUT" && $mom_nxt_motion_type == "FIRSTCUT") || ($mom_motion_type == "STEPOVER" && $mom_nxt_motion_type == "STEPOVER") } {
      if { [EQ_is_equal $PX $X] && [EQ_is_equal $PY $Y] && [EQ_is_equal $PZ $Z] && ![EQ_is_equal $P4 $R4] && [EQ_is_equal $P5 $R5] && [EQ_is_equal $NX $X] && [EQ_is_equal $NY $Y] && [EQ_is_equal $NZ $Z] && ![EQ_is_equal $N4 $R4] && [EQ_is_equal $N5 $R5] && $dpp_ge(combine_mode) != "5" && $switch_dir_4th == "NO" } {
         set dpp_ge(skip_move) "YES"

         MOM_force once fourth_axis

         set dpp_ge(combine_mode) "4"

         return

         } elseif { [EQ_is_equal $PX $X] && [EQ_is_equal $PY $Y] && [EQ_is_equal $PZ $Z] && [EQ_is_equal $P4 $R4] && ![EQ_is_equal $P5 $R5] && [EQ_is_equal $NX $X] && [EQ_is_equal $NY $Y] && [EQ_is_equal $NZ $Z] && [EQ_is_equal $N4 $R4] && ![EQ_is_equal $N5 $R5] && $dpp_ge(combine_mode) != "4" && $switch_dir_5th == "NO" } {
                  set dpp_ge(skip_move) "YES"

                  MOM_force once fifth_axis

                  set dpp_ge(combine_mode) "5"

                  return
                  }
      }

   set dpp_ge(combine_mode) "0"

   set last_4th_output $R4
   set last_5th_output $R5

   set last_4th_dir $cur_4th_dir
   set last_5th_dir $cur_5th_dir
   }
}


#=============================================================
proc PB_CMD_combine_rotary_init { } {
#=============================================================
# Comment out next line to enable combine-rotary mode
return

global mom_kin_read_ahead_next_motion

global dpp_ge

set dpp_ge(combine_mode) "0"

set mom_kin_read_ahead_next_motion "TRUE"

MOM_reload_kinematics
}


#=============================================================
proc PB_CMD_combine_rotary_output { } {
#=============================================================
global dpp_ge

if { [info exists dpp_ge(skip_move)] } {
   if { $dpp_ge(skip_move) == "YES" } {
      if { ![llength [ info commands MOM_abort_event ]] } {
         global mom_warning_info

         set mom_warning_info "MOM_abort_event is an invalid command.  Must use NX3 or later"

         MOM_catch_warning

         return
         }

      global mom_pos
      global mom_prev_pos
      global mom_mcs_goto
      global mom_prev_mcs_goto

      VMOV 5 mom_prev_pos mom_pos
      VMOV 3 mom_prev_mcs_goto mom_mcs_goto

      MOM_reload_variable -a mom_pos
      MOM_reload_variable -a mom_mcs_goto

      MOM_abort_event
      }
   }
}


#=============================================================
proc PB_CMD_coord_rotation_in_operation { } {
#=============================================================
# This command is used to detect rotary axis change inside operation for 3+2 milling.
# This command will output coordinate rotation code if the rotary axis change the position.
#
  global mom_kin_machine_type mom_operation_type
  global mom_tool_axis mom_tool_axis_type
  global mom_pos mom_prev_pos mom_mcs_goto mom_prev_mcs_goto
  global mom_prev_out_angle_pos mom_out_angle_pos
  global coord_offset dpp_ge mom_result
  global mom_sys_adjust_code mom_cutcom_adjust_register
  global mom_cycle_rapid_to_pos mom_cycle_retract_to_pos mom_cycle_feed_to_pos
  global mom_cycle_rapid_to mom_cycle_retract_to mom_cycle_feed_to
  global mom_arc_center mom_pos_arc_center

  if { ![string match "*5_axis*" $mom_kin_machine_type] } {
return
  }

  DPP_GE_GET_NCM_WORK_PLANE_CHANGE_MODE

  if { $dpp_ge(ncm_work_plane_change_mode) == "direct_change" } {
     if {$dpp_ge(coord_rot) != "NONE"} {
        MOM_output_literal "G69"
        MOM_output_literal "G49"
     }

     set dpp_ge(coord_rot) [DPP_GE_COOR_ROT "ZXZ" angle offset pos]
     for { set i 0 } { $i < 3 } { incr i } {
        set dpp_ge(coord_offset,$i) $offset($i)
        set dpp_ge(coord_rot_angle,$i) $angle($i)
        set dpp_ge(prev_coord_rot_angle,$i) $dpp_ge(coord_rot_angle,$i)
     }

     if { $dpp_ge(coord_rot) == "NONE" } {
        if { [info exists mom_cutcom_adjust_register] } {
           MOM_output_literal "G40"
           MOM_force once G_cutcom D
        }

        MOM_force once G_adjust H

        set dpp_ge(prev_coord_rot_angle,0) 0
        set dpp_ge(prev_coord_rot_angle,1) 0
        set dpp_ge(prev_coord_rot_angle,2) 0

       # If it's not auto3d condition, restore the kinematics and recalculate mom_pos
        DPP_GE_RESTORE_KINEMATICS
        PB_CMD__convert_point

        MOM_reload_variable -a mom_pos
        MOM_force Once fourth_axis fifth_axis
     } else {
        if { [info exists mom_cutcom_adjust_register] } {
           MOM_output_literal "G40"
           MOM_force once G_cutcom D
        }

        MOM_force once G_adjust H X Y Z fourth_axis fifth_axis

        VMOV 3 pos mom_pos
        MOM_reload_variable -a mom_pos
        MOM_do_template swiveling_coord_rot
        MOM_do_template auto_align_rotary_axis
        MOM_do_template three_plus_two_suppress CREATE
     }
  } elseif { $dpp_ge(ncm_work_plane_change_mode) != "None" } {
     if { $dpp_ge(ncm_work_plane_change_mode) == "start_change" } {
         # 5x non-cutting motion start
         # cancel tilt work plane
         if { $dpp_ge(coord_rot) != "NONE" } {
            MOM_output_literal "G69"
            MOM_output_literal "G49"
         }
         set dpp_ge(prev_coord_rot_angle,0)  0
         set dpp_ge(prev_coord_rot_angle,1)  0
         set dpp_ge(prev_coord_rot_angle,2)  0

         # output TCP mode
         set mom_sys_adjust_code 43.4
         set dpp_ge(fanuc_ncm_tcp) 1

         DPP_GE_RESTORE_KINEMATICS
         MOM_reload_variable -a mom_pos
         MOM_force once G_adjust H X Y Z fourth_axis fifth_axis
      }

      if { $dpp_ge(sys_output_coord_mode) == "TCP_FIX_TABLE" } {
         VMOV 3 mom_mcs_goto mom_pos
         VMOV 3 mom_prev_mcs_goto mom_prev_pos
         VMOV 3 mom_arc_center mom_pos_arc_center
      } else {
         PB_CMD__convert_point
      }
  } else {
     if { [info exists dpp_ge(fanuc_ncm_tcp)] && $dpp_ge(fanuc_ncm_tcp) == 1 } {
        #check if current output mode is TCP, if yes, then cancel it
        MOM_output_literal "G49"
        set mom_sys_adjust_code 43
        set dpp_ge(fanuc_ncm_tcp) 0

        #check if current work plane is tilted
        set dpp_ge(coord_rot) [DPP_GE_COOR_ROT "ZXZ" angle offset pos]
        for { set i 0 } { $i < 3 } { incr i } {
           set dpp_ge(coord_offset,$i) $offset($i)
           set dpp_ge(coord_rot_angle,$i) $angle($i)
           set dpp_ge(prev_coord_rot_angle,$i) $dpp_ge(coord_rot_angle,$i)
        }

        if { $dpp_ge(coord_rot) == "NONE" } {
           if {[info exists mom_cutcom_adjust_register]} {
              MOM_output_literal "G40"
              MOM_force once G_cutcom D
           }

           MOM_output_literal "G69"

           set dpp_ge(prev_coord_rot_angle,0) 0
           set dpp_ge(prev_coord_rot_angle,1) 0
           set dpp_ge(prev_coord_rot_angle,2) 0

          # If it's not auto3d condition, restore the kinematics and recalculate mom_pos
           DPP_GE_RESTORE_KINEMATICS
           PB_CMD__convert_point

           MOM_reload_variable -a mom_pos
           MOM_force Once G_adjust H fourth_axis fifth_axis
        } else {
           if { [info exists mom_cutcom_adjust_register] } {
              MOM_output_literal "G40"
              MOM_force once G_cutcom D
           }

           MOM_output_literal "G69"

           VMOV 3 pos mom_pos
           MOM_reload_variable -a mom_pos

           # Recalculate the hole parameters for this hole
           VMOV 3 mom_pos mom_cycle_rapid_to_pos
           VMOV 3 mom_pos mom_cycle_feed_to_pos
           VMOV 3 mom_pos mom_cycle_retract_to_pos

           set mom_cycle_rapid_to_pos(2)   [expr $mom_pos(2)+$mom_cycle_rapid_to]
           set mom_cycle_retract_to_pos(2) [expr $mom_pos(2)+$mom_cycle_retract_to]
           set mom_cycle_feed_to_pos(2)    [expr $mom_pos(2)+$mom_cycle_feed_to]

           MOM_do_template swiveling_coord_rot
           MOM_do_template auto_align_rotary_axis
           MOM_do_template three_plus_two_suppress CREATE
           MOM_force once G_adjust H X Y Z R
           # Reposition the tool to Initial level if tool axis has been changed between holes
           MOM_do_template cycle_plane_change_6
        }
     }
  }
}


#=============================================================
proc PB_CMD_create_tool_list { } {
#=============================================================
#  Place this custom command in either the start of program
#  or the end of program event marker to generate a tool list
#  in your NC file.
#
#  The Shop Doc template file "pb_post_tool_list.tpl" distributed with
#  Post Builder in "POSTBUILD/pblib/misc" directory can be copied
#  to the "mach/resource/postprocessor" or "mach/resource/shop_docs" directory,
#  in case that your UG runtime environment does not have access to the
#  Post Builder installation.
#
#  Accessing "pb_post_tool_list.tpl" in other location can also be accomplished
#  by changing the code below titled "Generate tool list data" in this proc.
#
#  The variable "mom_sys_tool_list_output_type" set in this proc allows you
#  to select the type of tool list to be generated.
#  The options are:
#
#   "ORDER_IN_USE"     - List tools used in the program in the order of operations.
#   "ALL_UNIQUE"       - List all unique tools once for each in the order of use.
#   "GROUP_BY_TYPE"    - List tools in groups of different tool types.
#
# The desired tool list type can be set by changing the code below.
# The default is set to "GROUP_BY_TYPE".

set wfl_global [info globals "mom_*"]

foreach gv $wfl_global {
        global Twfl_$gv $gv

        if { [info exists $gv] } {
           if { ![array exists $gv] } {
              set wfl Twfl_$gv
              set $wfl [set $gv]
              }
           }
        }

global mom_sys_tool_list_initialized
global mom_sys_tool_list_output_type

if { ![info exists mom_sys_tool_list_initialized] || !$mom_sys_tool_list_initialized } {
   MOM_output_to_listing_device "proc PB_CMD_init_tool_list must be executed in the Start of Program before PB_CMD_create_tool_list is called."

   return
   }

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Set mom_sys_tool_list_output_type to the desired output fashion.
#
#   "ORDER_IN_USE"     - List tools used in the program in the order of operations.
#   "ALL_UNIQUE"       - List all unique tools once for each in the order of use.
#   "GROUP_BY_TYPE"    - List tools in groups of different tool types.
#
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# set mom_sys_tool_list_output_type "ORDER_IN_USE"
# set mom_sys_tool_list_output_type "ALL_UNIQUE"
set mom_sys_tool_list_output_type "GROUP_BY_TYPE"

global mom_tool_name
global mom_tool_number
global mom_sys_control_in
global mom_sys_control_out
global mom_tool_length_adjust_register

global current_program_name

#----------------------------------------------------------------------------
# Save info for the currently active tool in the program being post-prcessed
# before starting Shop Doc mechanism for tool list generation.
#----------------------------------------------------------------------------
if [llength [info commands PB_CMD_save_active_oper_tool_data] ] {
   PB_CMD_save_active_oper_tool_data
   }

#-----------------------------------------------------------
# Create tool list per selected top-level group.
# Group name is set to blank if no group has been selected.
#-----------------------------------------------------------
global mom_parent_group_name

if [info exists mom_parent_group_name] {
   set current_program_name $mom_parent_group_name

   } else {
          set current_program_name ""
          }

set ci " "
set co " "

if [info exists mom_sys_control_in]  { set ci $mom_sys_control_in }
if [info exists mom_sys_control_out] { set co $mom_sys_control_out }

#*************************
# Generate tool list data
#*************************
set template_file pb_post_tool_list.tpl

global tcl_platform

if [string match "windows" $tcl_platform(platform)] {
   set pb_lib_misc_dir [MOM_ask_env_var UGII_BASE_DIR]\\postbuild\\pblib\\misc\\

   } else {
          set pb_lib_misc_dir [MOM_ask_env_var UGII_BASE_DIR]/postbuild/pblib/misc/
          }

set cam_post_dir     [MOM_ask_env_var UGII_CAM_POST_DIR]
set cam_shop_doc_dir [MOM_ask_env_var UGII_CAM_SHOP_DOC_DIR]

if { [file exists ${pb_lib_misc_dir}${template_file}] } {
   MOM_do_template_file ${pb_lib_misc_dir}${template_file}

   } elseif { [file exists ${cam_post_dir}${template_file}] } {
            MOM_do_template_file ${cam_post_dir}${template_file}

            } elseif { [file exists ${cam_shop_doc_dir}${template_file}] } {
                     MOM_do_template_file ${cam_shop_doc_dir}${template_file}

                     } else {
                            MOM_output_to_listing_device  "ERROR : Template file pb_post_tool_list.tpl is not found in the following directories: \n \n          $pb_lib_misc_dir \n          $cam_post_dir \n          $cam_shop_doc_dir \n \n        Tool list cannot be generated.\n"

                            return
                            }

#------------------
# Tool list header
#------------------
shop_doc_output_literal "$co==================================================================================================$ci"
shop_doc_output_literal "$co                                     T O O L   L I S T                                            $ci"
shop_doc_output_literal "$co==================================================================================================$ci"

#------------------
# Output tool list
#------------------
global mom_sys_tool_stack

global tool_data_buffer

switch $mom_sys_tool_list_output_type {
                         "ORDER_IN_USE" {
                                        set tool_list $mom_sys_tool_stack(IN_USE)
                                        }
                        "GROUP_BY_TYPE" {
                                        set tool_list [concat $mom_sys_tool_stack(LATHE)  $mom_sys_tool_stack(DRILL)  $mom_sys_tool_stack(MILL)]
                                        }
                                default {
                                        set tool_list $mom_sys_tool_stack(ALL)
                                        }
                                      }

set prev_tool_type ""

foreach tool $tool_list {
        set tool_type $tool_data_buffer($tool,type)

        # Output tool type header if it changes.
        if { ![string match "$tool_type" $prev_tool_type] } {
           if { [info exists tool_data_buffer($tool_type,header)] &&  $tool_data_buffer($tool_type,header) != "" } {
              shop_doc_output_literal "$tool_data_buffer($tool_type,header)"
              }
           }

        if [info exists tool_data_buffer($tool,output)] {
           shop_doc_output_literal "$tool_data_buffer($tool,output)"
           }

        set prev_tool_type $tool_type
        }

#------------------
# Tool list footer
#------------------
shop_doc_output_literal "$co                                                                                                  $ci"

#-------------------------------------------------------------------------------
# Restore info for the currently active tool in the program being post-prcessed.
#-------------------------------------------------------------------------------
if [llength [info commands PB_CMD_restore_active_oper_tool_data] ] {
   PB_CMD_restore_active_oper_tool_data
   }

set wfl_global [info globals "Twfl_*"]

foreach gv $wfl_global {
        set mv [string trimleft $gv "Twfl_"]

        global $gv $mv

        if { [info exists $gv] } {
           set $mv [set $gv]

           unset $gv
           }
        }
}


#=============================================================
proc PB_CMD_customize_output_mode { } {
#=============================================================
# This command is the port for user to chose the output TCP mode and 3+2 axis
# machining mode, change value of below variables to get different output
#
# 05-09-2013 levi - seperate this command from PB_CMD_set_default_dpp_value

global dpp_ge

## dpp_ge(sys_coord_rotation_output_type)
## "WCS_ROTATION"  G68
## "SWIVELING"     G68.2

## dpp_ge(sys_tcp_tool_axis_output_mode)
## "AXIS"    output the rotation angle of axis (G43.4)
## "VECTOR"  output tool axis vector(G43.5)

## dpp_ge(sys_output_coord_mode)
## "TCP_FIX_TABLE"    use a coordinate system fixed on the table as the programming coordinate system
## "TCP_FIX_MACHINE"  use workpiece coordinate system fixed on machine as the programming coordinate system

# Do customization here to get different output
set dpp_ge(sys_coord_rotation_output_type) "SWIVELING"
set dpp_ge(sys_tcp_tool_axis_output_mode) "AXIS"
set dpp_ge(sys_output_coord_mode) "TCP_FIX_TABLE" ; #this variable will be force changed to "TCP_FIX_TABLE" in postprocessor if dpp_ge(sys_tcp_tool_axis_output_mode) is set to "VECTOR"
}


#=============================================================
proc PB_CMD_cutcom_setting { } {
#=============================================================
# This command is to be called in linear move and circular move event to suppress
# G_plane address when the cutcom status has not changed. And assign value for mom_cutcom_adjust_register.
# -- Assuming G_cutcom address is modal and G_plane exists in the block
#
#<10-11-09 gsl> - New
#<01-20-11 gsl> - Force out plane code for the 1st linear move when CUTCOM is on
#<03-16-12 gsl> - Added use of CALLED_BY
#<06-06-13 levi> - Delete CALLED_BY. Change command name from PB_CMD_suppress_linear_block_plane_code.
#                  Add assign command for mom_cutcom_adjust_register.

global mom_cutcom_status
global mom_cutcom_register
global mom_cutcom_adjust_register

global mom_user_prev_cutcom_status

if { ![info exists mom_cutcom_status] } {
   set mom_cutcom_status UNDEFINED
   }

if { ![info exists mom_user_prev_cutcom_status] } {
   set mom_user_prev_cutcom_status UNDEFINED
   }

if { ![string match $mom_user_prev_cutcom_status $mom_cutcom_status] } {
   if { [string match "LEFT"  $mom_cutcom_status] || [string match "RIGHT" $mom_cutcom_status] || [string match "ON" $mom_cutcom_status] } {
      if { ![info exists mom_cutcom_adjust_register] && [info exists mom_cutcom_register] } {
         set mom_cutcom_adjust_register $mom_cutcom_register
         }

      MOM_force once D
      }

   set mom_user_prev_cutcom_status $mom_cutcom_status
   }
}


#=============================================================
proc PB_CMD_cycle_clearance_plane_change { } {
#=============================================================
# mom_cycle_tool_axis_change and mom_cycle_clearance_plane_change is added
# to detect the tool axis change and clearance plane change in cycles. They are only available
# in NX754 and later version.
# mom_cycle_tool_axis_change = 1 means tool axis is changed.
# mom_cycle_tool_axis_change = 0 means no tool axis change.
# mom_cycle_clearance_plane_change = 0 means no clearance plane change.
# mom_cycle_clearance_plane_change = 1 means clearance plane change from lower to higher.
# mom_cycle_clearance_plane_change = -1 means clearance plane change from higher to lower.

global mom_cycle_clearance_plane_change

global first_hole_flag

if { ![info exists first_hole_flag] } { set first_hole_flag "TRUE" }

if { [string match "TRUE" $first_hole_flag] } {
   set first_hole_flag "FALSE"

   return
   }

if { [string match "-1" $mom_cycle_clearance_plane_change] || \
     [string match  "1" $mom_cycle_clearance_plane_change] } {
   MOM_cycle_off
   MOM_do_template cycle_traverse
   }
}


#=============================================================
proc PB_CMD_cycle_hole_counter_reset { } {
#=============================================================
global pop_cycle_hole_counter
set    pop_cycle_hole_counter 0
}


#=============================================================
proc PB_CMD_def_work_plane { } {
#=============================================================
  global mom_sys_work_plane_change
  global mom_prev_out_angle_pos mom_out_angle_pos

  global rot_a rot_b rot_c delt_x delt_y delt_z
  global mom_kin_coordinate_system_type
  global mom_sys_coordinate_system_status

  if { ![info exists mom_prev_out_angle_pos(0)] } {
    set mom_prev_out_angle_pos(0)    0
    set mom_prev_out_angle_pos(1)    0
   }

if { ![EQ_is_equal $mom_out_angle_pos(0) $mom_prev_out_angle_pos(0)] || ![EQ_is_equal $mom_out_angle_pos(1) $mom_prev_out_angle_pos(1)] } {
    MOM_do_template rapid_rotary
  }
}


#=============================================================
proc PB_CMD_define_coolant_mode { } {
#=============================================================
global mom_coolant_status

if { [info exists mom_coolant_status] && [string match "AIRBLAST" $mom_coolant_status] } {
   set mom_coolant_status "TAP"
   }

if { ![info exists mom_coolant_status] || [string match "UNDEFINED" $mom_coolant_status] } {
   set mom_coolant_status "FLOOD"
   }
}


#=============================================================
proc PB_CMD_define_cycle_plane_change { } {
#=============================================================
# mom_cycle_tool_axis_change and mom_cycle_clearance_plane_change is added
# to detect the tool axis change and clearance plane change in cycles. They are only available
# in NX754 and later version.
# mom_cycle_tool_axis_change = 1 means tool axis is changed.
# mom_cycle_tool_axis_change = 0 means no tool axis change.
# mom_cycle_clearance_plane_change = 0 means no clearance plane change.
# mom_cycle_clearance_plane_change = 1 means clearance plane change from lower to higher.
# mom_cycle_clearance_plane_change = -1 means clearance plane change from higher to lower.

global mom_operation_type
global mom_cycle_retract_mode
global mom_cycle_retract_to_pos
global mom_cycle_clearance_plane_change

global save_mom_pos
global first_hole_flag

if { ![info exists first_hole_flag] } { set first_hole_flag "TRUE" }

if { [string match "TRUE" $first_hole_flag] } {
   set first_hole_flag "FALSE"

   return
   }

if { [string match "Point to Point" $mom_operation_type] && [string match "AUTO" $mom_cycle_retract_mode] } {
   set mom_cycle_retract_to_pos(2) $save_mom_pos(2)

   MOM_cycle_off
   MOM_do_template cycle_spindle
   MOM_do_template cycle_traverse

   return
   }

if { [string match "1" $mom_cycle_clearance_plane_change] } {
   MOM_cycle_off
   MOM_do_template cycle_spindle
   MOM_do_template cycle_traverse

   } elseif { [string match "-1" $mom_cycle_clearance_plane_change] } {
            MOM_cycle_off
            MOM_do_template cycle_spindle
            MOM_do_template cycle_traverse
            }
}


#=============================================================
proc PB_CMD_detect_tool_path_type { } {
#=============================================================
# Do the initial preparation here, including store machine kinematic parameters
# and detect tool path type.

global mom_kin_arc_output_mode
global mom_kin_helical_arc_output_mode

global dpp_ge
global mom_ude_5axis_tool_path

# Save original kinematics parameters defined by pb
DPP_GE_SAVE_KINEMATICS

# Detect tool path type, assign value for dpp_ge(toolpath_axis_num), it can be "3" or "5"
DPP_GE_DETECT_TOOL_PATH_TYPE

# If user use UDE to define tool path type, reassign the dpp value according to the ude variable.
if { [info exists mom_ude_5axis_tool_path] && $mom_ude_5axis_tool_path == "YES" } {
   set dpp_ge(toolpath_axis_num) "5"

   } elseif { [info exists mom_ude_5axis_tool_path] && $mom_ude_5axis_tool_path == "NO" } {
            set dpp_ge(toolpath_axis_num) "3"
            }

# If use 5 axis mode to output, divide all the arc and helix to line
if { $dpp_ge(toolpath_axis_num) == "5" } {
   set mom_kin_arc_output_mode         "LINEAR"
   set mom_kin_helical_arc_output_mode "LINEAR"

   MOM_reload_kinematics
   }
}


#=============================================================
proc PB_CMD_empty_line { } {
#=============================================================
MOM_output_literal " "
}


#=============================================================
proc PB_CMD_end_of_alignment_character { } {
#=============================================================
# This command restores sequnece number back to orignal
# This command may be used with the command "PM_CMD_start_of_alignment_character"
#
# 07-23-2012 yaoz - Initial version

  global mom_sys_leader saved_seq_num
  if [info exists saved_seq_num] {
    set mom_sys_leader(N) $saved_seq_num
  }
}


#=============================================================
proc PB_CMD_end_of_path { } {
#=============================================================
global mom_cycle_option mom_sys_nc_output_mode
global mom_next_oper_has_tool_change

global is_HPCC_mode
global pop_suppress_G49
if { $mom_sys_nc_output_mode == "PART" } {
if {[info exists pop_suppress_G49] && [EQ_is_equal $pop_suppress_G49 1] } {
set pop_suppress_G49 0
return
}
MOM_force once G_adjust
MOM_do_template TCP_off
}
if {[info exists is_HPCC_mode]} {
  if { $is_HPCC_mode == "ON" } {
    MOM_do_template HPCC_mode_off
    MOM_force ONCE Macro_call HPCC_mode
    MOM_do_template HPCC_mode_off
   }
}

catch { unset mom_cycle_option }
}


#=============================================================
proc PB_CMD_end_of_program { } {
#=============================================================

}


#=============================================================
proc PB_CMD_fifth_axis_rotate_move { } {
#=============================================================
#  This procedure is used by the ROTATE ude command to output a
#  fifth axis rotary move.  You can use the NC Data Definitions
#  section of postbuilder to modify the fifth_axis_rotary_move
#  block template.
#
#  Do NOT add this block to events or markers.  It is a static
#  block and it is not intended to be added to events.  Do NOT
#  change the name of this custom command.

MOM_force once fifth_axis
MOM_do_template fifth_axis_rotate_move
}


#=============================================================
proc PB_CMD_fix_RAPID_SET { } {
#=============================================================
# This command is provided to overwrite the system RAPID_SET
# (defined in ugpost_base.tcl) in order to correct the problem
# with workplane change that doesn't account for +/- directions
# along X or Y principal axis.  It also fixes the problem that
# the First Move was never identified correctly to force
# the output of the 1st point.
#
# The original command has been renamed as ugpost_RAPID_SET.
#
# - This command may be attached to the "Start of Program" event marker.
#
#
# Revisions:
#-----------
# 02-18-08 gsl - Initial version
# 02-26-09 gsl - Used mom_kin_machine_type to derive machine mode when it's UNDEFINED.
# 08-18-15 sws - PR7294525 : Use mom_current_motion to detect first move & initial move
#

  # Only redefine RAPID_SET once, since ugpost_base is only loaded once.
  #
   if { ![CMD_EXIST ugpost_RAPID_SET] } {
      if { [CMD_EXIST RAPID_SET] } {
         rename RAPID_SET ugpost_RAPID_SET
      }
   } else {
return
   }


#***********
uplevel #0 {

#====================
proc RAPID_SET { } {
#====================

   if { [CMD_EXIST PB_CMD_set_principal_axis] } {
      PB_CMD_set_principal_axis
   }


   global mom_cycle_spindle_axis mom_sys_work_plane_change
   global traverse_axis1 traverse_axis2 mom_motion_event mom_machine_mode
   global mom_pos mom_prev_pos mom_from_pos mom_last_pos mom_sys_home_pos
   global mom_sys_tool_change_pos
   global spindle_first rapid_spindle_inhibit rapid_traverse_inhibit
   global mom_current_motion


   if { ![info exists mom_from_pos($mom_cycle_spindle_axis)] && \
         [info exists mom_sys_home_pos($mom_cycle_spindle_axis)] } {

      set mom_from_pos(0) $mom_sys_home_pos(0)
      set mom_from_pos(1) $mom_sys_home_pos(1)
      set mom_from_pos(2) $mom_sys_home_pos(2)

   } elseif { ![info exists mom_sys_home_pos($mom_cycle_spindle_axis)] && \
              [info exists mom_from_pos($mom_cycle_spindle_axis)] } {

      set mom_sys_home_pos(0) $mom_from_pos(0)
      set mom_sys_home_pos(1) $mom_from_pos(1)
      set mom_sys_home_pos(2) $mom_from_pos(2)

   } elseif { ![info exists mom_sys_home_pos($mom_cycle_spindle_axis)] && \
             ![info exists mom_from_pos($mom_cycle_spindle_axis)] } {

      set mom_from_pos(0) 0.0 ; set mom_sys_home_pos(0) 0.0
      set mom_from_pos(1) 0.0 ; set mom_sys_home_pos(1) 0.0
      set mom_from_pos(2) 0.0 ; set mom_sys_home_pos(2) 0.0
   }

   if { ![info exists mom_sys_tool_change_pos($mom_cycle_spindle_axis)] } {
      set mom_sys_tool_change_pos($mom_cycle_spindle_axis) 100000.0
   }


   set is_initial_move [string match "initial_move" $mom_current_motion]
   set is_first_move   [string match "first_move"   $mom_current_motion]

   if { $is_initial_move || $is_first_move } {
      set mom_last_pos($mom_cycle_spindle_axis) $mom_sys_tool_change_pos($mom_cycle_spindle_axis)
   } else {
      if { [info exists mom_last_pos($mom_cycle_spindle_axis)] == 0 } {
         set mom_last_pos($mom_cycle_spindle_axis) $mom_sys_home_pos($mom_cycle_spindle_axis)
      }
   }


   if { $mom_machine_mode != "MILL" && $mom_machine_mode != "DRILL" } {
     # When machine mode is UNDEFINED, ask machine type
      if { ![string match "MILL" [PB_CMD_ask_machine_type]] } {
return
      }
   }


   WORKPLANE_SET

   set rapid_spindle_inhibit  FALSE
   set rapid_traverse_inhibit FALSE


   if { [EQ_is_lt $mom_pos($mom_cycle_spindle_axis) $mom_last_pos($mom_cycle_spindle_axis)] } {
      set going_lower 1
   } else {
      set going_lower 0
   }


   if { ![info exists mom_sys_work_plane_change] } {
      set mom_sys_work_plane_change 1
   }


  # Reverse workplane change direction per spindle axis
   global mom_spindle_axis

   if { [info exists mom_spindle_axis] } {

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # User can temporarily disable the work plane change for rapid moves along non-principal
    # spindle axis even when work plane change has been set in the Rapid Move event.
    #
    # Work plane change, if set, will still be in effect for moves along principal axes.
    #
    # - This flag has no effect if the work plane change is not set.
    #

      set disable_non_principal_spindle 0


      switch $mom_cycle_spindle_axis {
         0 {
            if [EQ_is_lt $mom_spindle_axis(0) 0.0] {
               set going_lower [expr abs($going_lower - 1)]
            }
         }
         1 {
            if [EQ_is_lt $mom_spindle_axis(1) 0.0] {
               set going_lower [expr abs($going_lower - 1)]
            }
         }
         2 {
         # Multi-spindle machine
            if [EQ_is_lt $mom_spindle_axis(2) 0.0] {
               set going_lower [expr abs($going_lower - 1)]
            }
         }
      }


     # Per user's choice above, disable work plane change for non-principal spindle axis
     #
      if { $disable_non_principal_spindle } {

         if { ![EQ_is_equal $mom_spindle_axis(0) 1] && \
              ![EQ_is_equal $mom_spindle_axis(1) 1] && \
              ![EQ_is_equal $mom_spindle_axis(0) 1] } {

            global mom_user_work_plane_change
            global mom_user_spindle_first

            set mom_user_work_plane_change $mom_sys_work_plane_change
            set mom_sys_work_plane_change 0

            if [info exists spindle_first] {
               set mom_user_spindle_first $spindle_first
            } else {
               set mom_user_spindle_first NONE
            }
         }
      }
   }


   if { $mom_sys_work_plane_change } {

      if { $going_lower } {
         set spindle_first FALSE
      } else {
         set spindle_first TRUE
      }

     # Force output in Initial Move and First Move.
      if { !$is_initial_move && !$is_first_move } {

         if { [EQ_is_equal $mom_pos($mom_cycle_spindle_axis) $mom_last_pos($mom_cycle_spindle_axis)] } {
            set rapid_spindle_inhibit TRUE
         } else {
            set rapid_spindle_inhibit FALSE
         }

         if { [EQ_is_equal $mom_pos($traverse_axis1) $mom_prev_pos($traverse_axis1)] && \
              [EQ_is_equal $mom_pos($traverse_axis2) $mom_prev_pos($traverse_axis2)] && \
              [EQ_is_equal $mom_pos(3) $mom_prev_pos(3)] && [EQ_is_equal $mom_pos(4) $mom_prev_pos(4)] } {

            set rapid_traverse_inhibit TRUE
         } else {
            set rapid_traverse_inhibit FALSE
         }
      }

   } else {
      set spindle_first NONE
   }

} ;# RAPID_SET

} ;# uplevel
#***********
}


#=============================================================
proc PB_CMD_fixture_offset { } {
#=============================================================
# This command will be called by PB_CMD__pattern_set_csys_start,
# PB_CMD__subprogram_output_end, MOM_initial_move & MOM_first_move
# to manage and output fixture offset instructions
#
  global mom_fixture_offset_value

   if { [CMD_EXIST PB_CMD_patch_fixture_offset_blocks] } {
      PB_CMD_patch_fixture_offset_blocks
   }

   if { ![info exists mom_fixture_offset_value] } {
      set mom_fixture_offset_value 0
   }

   if { [info exists mom_fixture_offset_value] } {
      if { $mom_fixture_offset_value < 0 } {
         set mom_fixture_offset_value 0
      }
      if { $mom_fixture_offset_value > 6 } {
         set mom_fixture_offset_value [expr $mom_fixture_offset_value - 6]
         MOM_do_template fixture_number_enhancement
return
      }
   }

   MOM_do_template fixture_number
}


#=============================================================
proc PB_CMD_force_cycle { } {
#=============================================================
  global cycle_init_flag
  global mom_cycle_rapid_to mom_cycle_retract_to

# if it's the first hole, force output the hole parameters
  if { [info exists cycle_init_flag] && [string match "TRUE" $cycle_init_flag] } {
      MOM_force once X Y Z R cycle_dwell cycle_step
  }

# if retract move has been output for last hole, force output the hole parameters
# for next hole
  if { [EQ_is_lt $mom_cycle_rapid_to $mom_cycle_retract_to] } {
      MOM_force once X Y Z R cycle_dwell cycle_step
  }
}


#=============================================================
proc PB_CMD_force_cycle_parameters { } {
#=============================================================
global force_cycle_parameters

if { ![info exists force_cycle_parameters] } { set force_cycle_parameters "TRUE" }

if { [string match "TRUE" $force_cycle_parameters] } {
   MOM_force once G_feed G_return G_motion G_tap X Y Z R F cycle_step cycle_dwell

   set force_cycle_parameters "FALSE"
   }
}


#=============================================================
proc PB_CMD_force_output { } {
#=============================================================
MOM_force once G_plane G_motion G_adjust G_feed G_mode X Y Z M_spindle H
}


#=============================================================
proc PB_CMD_fourth_axis_rotate_move { } {
#=============================================================
#  This procedure is used by the ROTATE ude command to output a
#  fourth axis rotary move.  You can use the NC Data Definitions
#  section of postbuilder to modify the fourth_axis_rotary_move
#  block template.
#
#  Do NOT add this block to events or markers.  It is a static
#  block and it is not intended to be added to events.  Do NOT
#  change the name of this custom command.

MOM_force once fourth_axis
MOM_do_template fourth_axis_rotate_move
}


#=============================================================
proc PB_CMD_handle_sync_event { } {
#=============================================================
global mom_sync_max
global mom_sync_code
global mom_sync_incr
global mom_sync_index
global mom_sync_start
global mom_sync_number

set mom_sync_start 99
set mom_sync_incr  1
set mom_sync_max   199

if { ![info exists mom_sync_code] } {
   set mom_sync_code $mom_sync_start
   }

set mom_sync_code [expr $mom_sync_code + $mom_sync_incr]

OPL "M[expr $mom_sync_number+99]"
}


#=============================================================
proc PB_CMD_header_operation { } {
#=============================================================
global mom_path_name
global mom_tool_name
global mom_tool_number
global mom_tool_diameter
global mom_spindle_speed
global mom_fixture_offset_value
global mom_tool_adjust_register
global mom_tool_cutcom_register
global mom_cutcom_adjust_register

global co ci

MOM_set_seq_off
OPL " "
OPL $co OPER: $mom_path_name $ci
MOM_set_seq_on

#check spindle speed and tool number
if { $mom_spindle_speed == 0 } {
   PAUSE \
   "Spindle Speed Zero in operation $mom_path_name\. Please set Spindle Speed."
#   MOM_abort \
#   "Spindle Speed Zero in operation $mom_path_name\. Please set Spindle Speed."

   } elseif { $mom_tool_number == 0 } {
            PAUSE \
            "Tool Number Zero in tool $mom_tool_name\. Please set Tool Number."
#            MOM_abort \
#            "Tool Number Zero in tool $mom_tool_name\. Please set Tool Number."
            }

if { ![info exists mom_fixture_offset_value] || [EQ_is_zero $mom_fixture_offset_value] } {
   set mom_fixture_offset_value 1
   }

if { ![info exists mom_tool_adjust_register] || [EQ_is_zero $mom_tool_adjust_register] } {
   set mom_tool_adjust_register $mom_tool_number
   }

if { ![info exists mom_tool_cutcom_register] || [EQ_is_zero $mom_tool_cutcom_register] } {
   set mom_tool_cutcom_register $mom_tool_number
   }

set mom_cutcom_adjust_register $mom_tool_cutcom_register
}


#=============================================================
proc PB_CMD_helix_move { } {
#=============================================================
global PI
global mom_prev_pos
global mom_helix_pitch
global mom_pos_arc_plane
global mom_sys_cir_vector
global mom_pos_arc_center
global mom_sys_helix_pitch_type

switch $mom_pos_arc_plane {
                         XY { MOM_suppress once K ; set cir_index 2 }
                         YZ { MOM_suppress once I ; set cir_index 0 }
                         ZX { MOM_suppress once J ; set cir_index 1 }
                          }

switch $mom_sys_helix_pitch_type {
                              none { }
                   rise_revolution { set pitch $mom_helix_pitch }
                       rise_radian { set pitch [expr $mom_helix_pitch / ($PI * 2.0)]}
                             other {
                                   #Place your custom helix pitch code here
                                   }
                           default { set mom_sys_helix_pitch_type "none" }
                                 }

if { [string compare "none" $mom_sys_helix_pitch_type] } {
   switch $mom_sys_cir_vector {
                      "Vector - Arc Center to Start" {
                                                     set mom_prev_pos($cir_index) $pitch
                                                     set mom_pos_arc_center($cir_index) 0.0
                                                     }
                      "Vector - Arc Start to Center" -
             "Unsigned Vector - Arc Start to Center" {
                                                     set mom_prev_pos($cir_index) 0.0
                                                     set mom_pos_arc_center($cir_index) $pitch
                                                     }
                      "Vector - Absolute Arc Center" {
                                                     set mom_pos_arc_center($cir_index) $pitch
                                                     }
                              }
   }

# You may need to edit this line if you output more than one block
# or if you have changed the name of your circular_move block template

MOM_suppress once K
MOM_force once G_motion X Y Z I J

MOM_do_template circular_move
}


#=============================================================
proc PB_CMD_history_changes { } {
#=============================================================
global usr_data_rev
global usr_controle_rev

set usr_controle_rev "-"
set usr_data_rev     "17/MAY/2017"
#------------------------------------------------------------------------------------

#  History Changes
#=====================================================================================
#  Date        Rev    Changed By      Description
#=====================================================================================
#-------------------------------------------------------------------------------------
# 17/MAY/2017   -     Jean            - Initial Release;
#-------------------------------------------------------------------------------------
}


#=============================================================
proc PB_CMD_init_after_first_tool { } {
#=============================================================
   MOM_force once G_mode G Z
   MOM_do_template auto_tool_change_1
   MOM_do_template caxis_unclamp
   MOM_force once G_mode G fifth_axis
   MOM_do_template return_c
   MOM_do_template caxis_clamp
   MOM_do_template caxis_unclamp_1
   MOM_force once G_mode G fourth_axis
   MOM_do_template auto_tool_change_2
   MOM_do_template caxis_clamp_1
   MOM_force once G_mode G X
   MOM_do_template auto_tool_change_3
   MOM_force once G_mode G Y
   MOM_do_template auto_tool_change_4
   MOM_force once G_mode G Z
   MOM_do_template auto_tool_change_5
   MOM_do_template opstop
   MOM_force once T
   MOM_do_template auto_tool_change
   MOM_do_template tool_change_1
   MOM_force once X Y Z fourth_axis fifth_axis
}


#=============================================================
proc PB_CMD_init_before_first_tool { } {
#=============================================================
   global mom_kin_machine_type

   MOM_force once G_spin G_feed M_spindle
   MOM_do_template auto_tool_change_6
   MOM_do_template turning_mode_off
   MOM_do_template spindle_off

   if { [string match "5_axis*" $mom_kin_machine_type] || \
        [string match "4_axis*" $mom_kin_machine_type] || \
        [string match "*mill_turn" $mom_kin_machine_type] } \
   {
      MOM_do_template auto_tool_change_7
   }
}


#=============================================================
proc PB_CMD_init_force_address { } {
#=============================================================
# Force output address in initial move and first move.

MOM_force once G_mode G_adjust S M_spindle X Y Z F H
}


#=============================================================
proc PB_CMD_init_helix { } {
#=============================================================
#
# This procedure will be executed automatically at the start of program and
# anytime it is loaded as a slave post of a linked post.
#
# - This procedure can be used to enable your post to output helix.
#   You can choose from the following options to format the circle
#   block template to output the helix parameters.
#

uplevel #0 {

   set mom_sys_helix_pitch_type    "rise_radian"

#
# The default setting for mom_sys_helix_pitch_type is "rise_radian".
# This is the most common.  Other choices are:
#
#    "rise_radian"              Measures the rise over one radian.
#    "rise_revolution"          Measures the rise over 360 degrees.
#    "none"                     Will suppress the output of pitch.
#    "other"                    Allows you to calculate the pitch
#                               using your own formula.
#
# This custom command uses the block template circular_move to output
# the helix block.  If your post uses a block template with a different
# name, you must edit the line that outputs the helix block.

#
#  The following variable deines the output mode for helical records.
#
#  FULL_CIRCLE  -- This mode will output a helix record for each 360
#                  degrees of the helix.
#  QUADRANT  --    This mode will output a helix record for each 90
#                  degrees of the helix.
#  LINEAR  --      This mode will output the entire helix as linear gotos.
#  END_POINT --    This mode will assume the control can define an entire
#                  helix in a single block.

   set mom_kin_helical_arc_output_mode FULL_CIRCLE

   MOM_reload_kinematics

} ;# uplevel
}


#=============================================================
proc PB_CMD_init_helix_custom { } {
#=============================================================
# This procedure will be executed automatically at the start of program and
# anytime it is loaded as a slave post of a linked post.
#
# This procedure can be used to enable your post to output helix.
# You can choose from the following options to format the circle
# block template to output the helix parameters.

uplevel #0 {

   set mom_sys_helix_pitch_type    "rise_radian"

   # The default setting for mom_sys_helix_pitch_type is "rise_radian".
   # This is the most common.  Other choices are:
   #
   #    "rise_radian"              Measures the rise over one radian.
   #    "rise_revolution"          Measures the rise over 360 degrees.
   #    "none"                     Will suppress the output of pitch.
   #    "other"                    Allows you to calculate the pitch
   #                               using your own formula.
   #
   # This custom command uses the block template circular_move to output
   # the helix block.  If your post uses a block template with a different
   # name, you must edit the line that outputs the helix block.

   #
   #  The following variable deines the output mode for helical records.
   #
   #  FULL_CIRCLE  -- This mode will output a helix record for each 360
   #                  degrees of the helix.
   #  QUADRANT  --    This mode will output a helix record for each 90
   #                  degrees of the helix.
   #  LINEAR  --      This mode will output the entire helix as linear gotos.
   #  END_POINT --    This mode will assume the control can define an entire
   #                  helix in a single block.

   global mom_operation_type

   if { [string match "Cylinder Milling" $mom_operation_type] } {
      set mom_kin_helical_arc_output_mode FULL_CIRCLE

      } else {
             set mom_kin_helical_arc_output_mode LINEAR
             }

   MOM_reload_kinematics

   } ; #uplevel
}


#=============================================================
proc PB_CMD_init_rotary { } {
#=============================================================
uplevel #0 {
   # Retract and Re-Engage Parameters
   #
   # This option is activated by setting the Axis Limit Violation
   # Handling option on the Machine Tool dialog to Retract/Re-Engage.
   #
   # The sequence of actions that take place when a rotary limit violation
   # occurs is a retract to the clearance geometry at the rapid feedrate,
   # reposition the rotary axes so they do not violate, move back to
   # the engage point at the retract feedrate and engage into the part again.
   #
   # You can set additional parameters that will control the retract
   # and re-engage motion.
   #
   #
   #  mom_kin_retract_type ------- specifies the method used to
   #                               calculate the retract point.
   #                               The method can be of
   #
   #    DISTANCE : The retract will be to a point at a fixed distance
   #               along the spindle axis.
   #
   #    SURFACE  : For a 4-axis rotary head machine, the retract will
   #               be to a cylinder.  For a 5-axis dual heads machine,
   #               the retract will be to a sphere.  For machine with
   #               only rotary table(s), the retract will be to a plane
   #               normal & along the spindle axis.
   #
   #  mom_kin_retract_distance --- specifies the distance or radius for
   #                               defining the geometry of retraction.
   #
   #  mom_kin_reengage_distance -- specifies the re-engage point above
   #                               the part.

   set mom_kin_retract_type      "DISTANCE"
   set mom_kin_retract_distance  10.0
   set mom_kin_reengage_distance .20

   # The following parameters are used by UG Post.  Do NOT change
   # them unless you know what you are doing.
   if { ![info exists mom_kin_spindle_axis] } {
      set mom_kin_spindle_axis(0) 0.0
      set mom_kin_spindle_axis(1) 0.0
      set mom_kin_spindle_axis(2) 1.0
      }

   set spindle_axis_defined 1

   if { ![info exists mom_sys_spindle_axis] } {
      set spindle_axis_defined 0

      } else {
             if { ![array exists mom_sys_spindle_axis] } {
                unset mom_sys_spindle_axis

                set spindle_axis_defined 0
                }
             }

   if !$spindle_axis_defined {
      set mom_sys_spindle_axis(0) 0.0
      set mom_sys_spindle_axis(1) 0.0
      set mom_sys_spindle_axis(2) 1.0
      }

   set mom_sys_lock_status "OFF"

   } ; #uplevel
}


#=============================================================
proc PB_CMD_init_tool_list { } {
#=============================================================
#  This command will be executed automatically at the "Start of Program" to
#  prepare for the tool list generation.
#
#  This command will add the shop doc event handlers to the post.
#  You may edit the proc MOM_TOOL_BODY to customize your tool list output.
#
#  Only the tools used in the program being post-processed will be listed.
#
#  In order to create the tool list, you MUST add the command
#  PB_CMD_create_tool_list to either the "Start of Program"
#  or the "End of Program" event marker depending on where
#  the tool list is to be output in your NC code.
#
#  The Shop Doc template file "pb_post_tool_list.tpl" residing in the
#  "postbuild/pblib/misc" directory is required for this service to work.
#  You may need to copy it to the "mach/resource/postprocessor"
#  or "mach/resource/shop_docs" directory, in case your UG runtime
#  environment does not have access to the Post Builder installation.

global mom_sys_tool_list_initialized

uplevel #0 {

   proc MOM_TOOL_BODY { } {
      global mom_tool_name
      global mom_tool_type
      global mom_tool_number
      global mom_tool_length
      global mom_tool_diameter
      global mom_sys_tool_stack
      global mom_sys_control_in
      global mom_sys_control_out
      global mom_tool_nose_radius
      global mom_template_subtype
      global mom_tool_point_angle
      global mom_tool_orientation
      global mom_tool_flute_length
      global mom_tool_corner1_radius
      global mom_tool_lower_corner_radius
      global mom_tool_length_adjust_register

      global tool_data_buffer
      global cycle_program_name
      global current_program_name

      # Handle single operation case.
      # current_program_name will be blank when no group has been selected.

      if { $current_program_name != "" } {
         set n1 [string toupper $cycle_program_name]
         set n2 [string toupper $current_program_name]

         if { $n1 != $n2 && $n1 != "" } {
            return
            }

         } else {
                # mom_sys_change_mach_operation_name is set in MOM_machine_mode
                # Use this variable to generate tool info for a single operation.

                global mom_sys_change_mach_operation_name mom_operation_name

                if [info exists mom_sys_change_mach_operation_name] {
                   if { ![string match "$mom_operation_name" $mom_sys_change_mach_operation_name] } {
                      return
                      }

                   } else {
                          return
                          }
                }

      #****************************
      # Collect various tool lists
      #****************************
      lappend mom_sys_tool_stack(IN_USE) $mom_tool_name

      set tool_type [MAP_TOOL_TYPE]

      if { [lsearch $mom_sys_tool_stack(ALL) $mom_tool_name] < 0 } {
         lappend mom_sys_tool_stack(ALL)         $mom_tool_name
         lappend mom_sys_tool_stack($tool_type)  $mom_tool_name
         }

      #*************************************************
      # Define data to be output for each tool per type
      #*************************************************
      set output ""

      set ci $mom_sys_control_in
      set co $mom_sys_control_out

      if { $mom_template_subtype == "" } { set mom_template_subtype $mom_tool_type }

      set tool_name [string range $mom_tool_name 0 19]
      set template_subtype [string range $mom_template_subtype 0 19]

      if { ![info exists mom_tool_length_adjust_register] || $mom_tool_length_adjust_register == 0 } {
         set mom_tool_length_adjust_register $mom_tool_number
         }

      if { [string match "*-T Cutter" $mom_tool_type] } {
         set mom_tool_corner1_radius $mom_tool_lower_corner_radius
         }

      if { ![info exists mom_tool_corner1_radius] } {
         set mom_tool_corner1_radius 0.0
         }

      switch $tool_type {
                   "MILL" {
                          set output [format "%-20s %-14s %-20s %-10.4f %-10.4f %-10.4f %-8d"  $tool_name $mom_tool_number $template_subtype  $mom_tool_diameter $mom_tool_corner1_radius  $mom_tool_flute_length $mom_tool_length_adjust_register]
                          }
                  "DRILL" {
                          set mom_tool_point_angle [expr (180.0 / 3.14159) * $mom_tool_point_angle]
                          set output [format "%-20s %-14s %-20s %-10.4f %-10.4f %-10.4f %-8d"  $tool_name $mom_tool_number $template_subtype  $mom_tool_diameter $mom_tool_point_angle  $mom_tool_flute_length $mom_tool_length_adjust_register]
                          }
                  "LATHE" {
                          set pi [expr 2 * asin(1.0)]
                          set tool_orient [expr (180. / 3.14159) * $mom_tool_orientation]
                          set output [format "%-20s %-14s %-20s %-10.4f %-15.4f %-8d"  $tool_name $mom_tool_numer $template_subtype  $mom_tool_nose_radius $tool_orient  $mom_tool_length_adjust_register]
                          }
                        }

      #*******************************************************************************
      # Fetch tool time data from the post.
      # This info is only available when tool list is created at the end of a program.
      #*******************************************************************************
      global mom_sys_tool_time
      global mom_operation_name
      global mom_sys_tool_list_output_type

      set tool_time ""

      if [info exists mom_sys_tool_time] {
         switch $mom_sys_tool_list_output_type {
                                  "ORDER_IN_USE" {
                                                 # Tool time per operations.
                                                 set tool_time $mom_sys_tool_time($mom_tool_name,$mom_operation_name)
                                                 }
                                         default {
                                                 # Accumulate tool time from all operations using this tool.
                                                 set tool_time 0
                                                 if [info exists mom_sys_tool_time($mom_tool_name,oper_list)] {
                                                    foreach oper $mom_sys_tool_time($mom_tool_name,oper_list) {
                                                            set tool_time [expr $tool_time + $mom_sys_tool_time($mom_tool_name,$oper)]
                                                            }
                                                    }
                                                 }
                                               }
         }

      if { $tool_time != ""  &&  $tool_time != "0" } {
         set tool_time [format "%-10.2f" $tool_time]
         }

      #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      # Store data to be output or used in PB_CMD_create_tool_list.
      #
      # <Ex.>
      #  global mom_tool_number
      #   set tool_data_buffer($mom_tool_name,tool_number) $mom_tool_number
      #
      # If a BLOCK_TEMPLATE is used to output the data, the global varaibles
      # used in the expression of an Address need to be set accordingly
      # before "MOM_do_template" is called.
      #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      set tool_data_buffer($mom_tool_name,output) "$co$output$tool_time$ci"
      set tool_data_buffer($mom_tool_name,type)   "$tool_type"
      }

   proc MOM_SETUP_HDR {} {
      global mom_sys_control_in
      global mom_sys_control_out

      # Initialize various tool lists
      global mom_sys_tool_stack

      set mom_sys_tool_stack(IN_USE) [list]
      set mom_sys_tool_stack(ALL)    [list]
      set mom_sys_tool_stack(MILL)   [list]
      set mom_sys_tool_stack(DRILL)  [list]
      set mom_sys_tool_stack(LATHE)  [list]

      set ci $mom_sys_control_in
      set co $mom_sys_control_out

      #++++++++++++++++++++++++++++++++++++++++++
      # Define header to be output per tool type
      #++++++++++++++++++++++++++++++++++++++++++
      global tool_data_buffer

      set tool_number "TOOL NUMBER"
      set tool_desc   "DESCRIPTION"
      set tool_dia    "DIAMETER"
      set corner_rad  "COR RAD"
      set tip_ang     "TIP ANG"
      set flute_len   "FLUTE LEN"
      set adjust      "ADJ REG"
      set nose_dia    "NOSE RAD"
      set tool_orient "TOOL ORIENT"

      # Label title for tool time only when it exists.
      global mom_sys_tool_time

      if [info exists mom_sys_tool_time] {
         set mach_time "MACH TIME"

         } else {
                set mach_time ""
                }

      set tool_name "DRILL"
      set output [format "%-20s %-14s %-20s %-10s %-10s %-10s %-7s %-0s"  $tool_name $tool_number $tool_desc $tool_dia $tip_ang $flute_len $adjust $mach_time]

      set header [list]

      lappend header "$co                                                                                                  $ci"
      lappend header "$co--------------------------------------------------------------------------------------------------$ci"
      lappend header "$co$output$ci"
      lappend header "$co--------------------------------------------------------------------------------------------------$ci"

      set tool_data_buffer(DRILL,header) [join $header \n]

      set tool_name   "MILL"
      set output [format "%-20s %-14s %-20s %-10s %-10s %-10s %-7s %-0s"  $tool_name $tool_number $tool_desc $tool_dia $corner_rad $flute_len $adjust $mach_time]

      set header [list]

      lappend header "$co                                                                                                  $ci"
      lappend header "$co--------------------------------------------------------------------------------------------------$ci"
      lappend header "$co$output$ci"
      lappend header "$co--------------------------------------------------------------------------------------------------$ci"

      set tool_data_buffer(MILL,header) [join $header \n]

      set tool_name   "LATHE"
      set output [format "%-20s %-14s %-20s %-10s %-15s %-7s %-0s"  $tool_name $tool_number $tool_desc $nose_dia $tool_orient $adjust $mach_time]

      set header [list]

      lappend header "$co                                                                                                  $ci"
      lappend header "$co--------------------------------------------------------------------------------------------------$ci"
      lappend header "$co$output$ci"
      lappend header "$co--------------------------------------------------------------------------------------------------$ci"

      set tool_data_buffer(LATHE,header) [join $header \n]
      }


   proc MOM_PROGRAMVIEW_HDR {} {}

   proc MOM_SETUP_FTR {} {}

   proc MOM_MEMBERS_HDR {} {
      global mom_sys_program_stack

      global cycle_program_name
      global current_program_name

      lappend mom_sys_program_stack $cycle_program_name

      if { [lsearch $mom_sys_program_stack "$current_program_name"] >= 0 } {
         set cycle_program_name $current_program_name
         }
      }


   proc MOM_MEMBERS_FTR {} {
      global mom_sys_program_stack

      global cycle_program_name
      global current_program_name

      set mom_sys_program_stack [lreplace $mom_sys_program_stack end end]
      set cycle_program_name [lindex $mom_sys_program_stack end]

      if { [lsearch $mom_sys_program_stack "$current_program_name"] >= 0 } {
         set cycle_program_name $current_program_name
         }
      }

   proc MOM_PROGRAM_BODY {} {
      global mom_object_name

      global cycle_program_name

      set cycle_program_name $mom_object_name
      }

   proc MOM_SETUP_BODY {} {}

   proc MOM_OPER_BODY  {} {}

   proc MOM_TOOL_HDR   {} {}

   proc MOM_TOOL_FTR   {} {}

   proc MOM_PROGRAMVIEW_FTR {} {}

   proc MAP_TOOL_TYPE { } {
      global mom_tool_type

      if { [string match "Milling*" $mom_tool_type] } {
         return "MILL"

         } elseif { [string match "Turning*" $mom_tool_type] } {
                  return "LATHE"

                  } elseif { [string match "Grooving*" $mom_tool_type] } {
                           return "LATHE"

                           } elseif { [string match "Threading*" $mom_tool_type] } {
                                    return "LATHE"

                                    } elseif { [string match "Drilling*" $mom_tool_type] } {
                                             return "DRILL"

                                             } else {
                                                    return "DRILL"
                                                    }
      }

   proc shop_doc_output_literal { line } {
      global list_file
      global tool_list_commentary

      set line_list [split $line \n]

      foreach line $line_list {
              if [info exists tool_list_commentary] {
                 puts $list_file $line

                 } else {
                        OPL $line
                        }

              }
      }
   } ; # uplevel


set mom_sys_tool_list_initialized 1
}


#=============================================================
proc PB_CMD_initial_prog_variables { } {
#=============================================================
global dpp_ge co ci

set co "("
set ci ")"

set dpp_ge(prev_toolpath_axis_num) "UNDEFINED"
}


#=============================================================
proc PB_CMD_kin__MOM_lintol { } {
#=============================================================
   global mom_kin_linearization_flag
   global mom_kin_linearization_tol
   global mom_lintol_status
   global mom_lintol

   if { ![string compare "ON" $mom_lintol_status] } {
      set mom_kin_linearization_flag "TRUE"
      if { [info exists mom_lintol] } {
         set mom_kin_linearization_tol $mom_lintol
      }
   } elseif { ![string compare "OFF" $mom_lintol_status] } {
      set mom_kin_linearization_flag "FALSE"
   }
}


#=============================================================
proc PB_CMD_kin__MOM_rotate { } {
#=============================================================
# This command handles a Rotate UDE.
#
# Key parameters set in UDE -
#   mom_rotate_axis_type        :  [ AAXIS | BAXIS   | CAXIS    | HEAD | TABLE | FOURTH_AXIS | FIFTH_AXIS ]
#   mom_rotation_mode           :  [ NONE  | ATANGLE | ABSOLUTE | INCREMENTAL ]
#   mom_rotation_direction      :  [ NONE  | CLW     | CCLW ]
#   mom_rotation_angle          :  Specified angle
#   mom_rotation_reference_mode :  [ ON    | OFF ]
#
#
## <rws 04-11-2008>
## If in TURN mode and user invokes "Flip tool around Holder" a MOM_rotate event is generated
## When this happens ABORT this event via return
##
## 09-12-2013 gsl - Made code & functionality of MOM_rotate sharable among all multi-axis posts.
##

   global mom_machine_mode


   if { [info exists mom_machine_mode] && [string match "TURN" $mom_machine_mode] } {
      if { [CMD_EXIST PB_CMD_handle_lathe_flash_tool] } {
         PB_CMD_handle_lathe_flash_tool
      }
return
   }


   global mom_rotate_axis_type mom_rotation_mode mom_rotation_direction
   global mom_rotation_angle mom_rotation_reference_mode
   global mom_kin_machine_type mom_kin_4th_axis_direction mom_kin_5th_axis_direction
   global mom_kin_4th_axis_leader mom_kin_5th_axis_leader
   global mom_kin_4th_axis_leader mom_kin_5th_axis_leader mom_pos
   global mom_out_angle_pos
   global unlocked_prev_pos mom_sys_leader
   global mom_kin_4th_axis_min_limit mom_kin_4th_axis_max_limit
   global mom_kin_5th_axis_min_limit mom_kin_5th_axis_max_limit
   global mom_prev_pos
   global mom_prev_rot_ang_4th mom_prev_rot_ang_5th


   if { ![info exists mom_rotation_angle] } {
     # Should the event be aborted here???
return
   }


   if { ![info exists mom_kin_5th_axis_direction] } {
      set mom_kin_5th_axis_direction "0"
   }


  #
  #  Determine which rotary axis the UDE has specifid - fourth(3), fifth(4) or invalid(0)
  #
  #
   if { [string match "*3_axis_mill_turn*" $mom_kin_machine_type] } {

      switch $mom_rotate_axis_type {
         CAXIS -
         FOURTH_AXIS -
         TABLE {
            set axis 3
         }
         default {
            set axis 0
         }
      }

   } else {

      switch $mom_rotate_axis_type {
         AAXIS -
         BAXIS -
         CAXIS {
            set axis [AXIS_SET $mom_rotate_axis_type]
         }
         HEAD {
            if { ![string compare "5_axis_head_table" $mom_kin_machine_type] ||\
                 ![string compare "5_AXIS_HEAD_TABLE" $mom_kin_machine_type] } {
               set axis 4
            } else {
               set axis 3
            }
         }
         FIFTH_AXIS {
            set axis 4
         }
         FOURTH_AXIS -
         TABLE -
         default {
            set axis 3
         }
      }
   }

   if { $axis == 0 } {
      CATCH_WARNING "Invalid rotary axis ($mom_rotate_axis_type) has been specified."
      MOM_abort_event
   }

   switch $mom_rotation_mode {
      NONE -
      ATANGLE {
         set angle $mom_rotation_angle
         set mode 0
      }
      ABSOLUTE {
         set angle $mom_rotation_angle
         set mode 1
      }
      INCREMENTAL {
         set angle [expr $mom_pos($axis) + $mom_rotation_angle]
         set mode 0
      }
   }

   switch $mom_rotation_direction {
      NONE {
         set dir 0
      }
      CLW {
         set dir 1
      }
      CCLW {
         set dir -1
      }
   }

   set ang [LIMIT_ANGLE $angle]
   set mom_pos($axis) $ang

   if { $axis == "3" } { ;# Rotate 4th axis

      if { ![info exists mom_prev_rot_ang_4th] } {
         set mom_prev_rot_ang_4th [MOM_ask_address_value fourth_axis]
      }
      if { [string length [string trim $mom_prev_rot_ang_4th]] == 0 } {
         set mom_prev_rot_ang_4th 0.0
      }

      set prev_angles(0) $mom_prev_rot_ang_4th

   } elseif { $axis == "4" } { ;# Rotate 5th axis

      if { ![info exists mom_prev_rot_ang_5th] } {
         set mom_prev_rot_ang_5th [MOM_ask_address_value fifth_axis]
      }
      if { [string length [string trim $mom_prev_rot_ang_5th]] == 0 } {
         set mom_prev_rot_ang_5th 0.0
      }

      set prev_angles(1) $mom_prev_rot_ang_5th
   }

   set p [expr $axis + 1]
   set a [expr $axis - 3]

   if { $axis == 3  &&  [string match "MAGNITUDE_DETERMINES_DIRECTION" $mom_kin_4th_axis_direction] } {

      set dirtype "MAGNITUDE_DETERMINES_DIRECTION"

      global mom_sys_4th_axis_dir_mode

      if { [info exists mom_sys_4th_axis_dir_mode] && ![string compare "ON" $mom_sys_4th_axis_dir_mode] } {

         set del $dir
         if { $del == 0 } {
            set del [expr $ang - $mom_prev_pos(3)]
            if { $del >  180.0 } { set del [expr $del - 360.0] }
            if { $del < -180.0 } { set del [expr $del + 360.0] }
         }

         global mom_sys_4th_axis_cur_dir
         global mom_sys_4th_axis_clw_code mom_sys_4th_axis_cclw_code

         if { $del > 0.0 } {
            set mom_sys_4th_axis_cur_dir $mom_sys_4th_axis_clw_code
         } elseif { $del < 0.0 } {
            set mom_sys_4th_axis_cur_dir $mom_sys_4th_axis_cclw_code
         }
      }

   } elseif { $axis == 4  &&  [string match "MAGNITUDE_DETERMINES_DIRECTION" $mom_kin_5th_axis_direction] } {

      set dirtype "MAGNITUDE_DETERMINES_DIRECTION"

      global mom_sys_5th_axis_dir_mode

      if { [info exists mom_sys_5th_axis_dir_mode] && ![string compare "ON" $mom_sys_5th_axis_dir_mode] } {

         set del $dir
         if { $del == 0 } {
            set del [expr $ang - $mom_prev_pos(4)]
            if { $del >  180.0 } { set del [expr $del - 360.0] }
            if { $del < -180.0 } { set del [expr $del + 360.0] }
         }

         global mom_sys_5th_axis_cur_dir
         global mom_sys_5th_axis_clw_code mom_sys_5th_axis_cclw_code

         if { $del > 0.0 } {
            set mom_sys_5th_axis_cur_dir $mom_sys_5th_axis_clw_code
         } elseif { $del < 0.0 } {
            set mom_sys_5th_axis_cur_dir $mom_sys_5th_axis_cclw_code
         }
      }

   } else {

      set dirtype "SIGN_DETERMINES_DIRECTION"
   }

   if { $mode == 1 } {

      set mom_out_angle_pos($a) $angle

   } elseif { [string match "MAGNITUDE_DETERMINES_DIRECTION" $dirtype] } {

      if { $axis == 3 } {
         set mom_out_angle_pos($a) [ROTSET $ang $prev_angles(0) $mom_kin_4th_axis_direction\
                                                $mom_kin_4th_axis_leader mom_sys_leader(fourth_axis)\
                                                $mom_kin_4th_axis_min_limit $mom_kin_4th_axis_max_limit]
      } else {
         set mom_out_angle_pos($a) [ROTSET $ang $prev_angles(1) $mom_kin_5th_axis_direction\
                                                $mom_kin_5th_axis_leader mom_sys_leader(fifth_axis)\
                                                $mom_kin_5th_axis_min_limit $mom_kin_5th_axis_max_limit]
      }

   } elseif { [string match "SIGN_DETERMINES_DIRECTION" $dirtype] } {

      if { $dir == -1 } {
         if { $axis == 3 } {
            set mom_sys_leader(fourth_axis) $mom_kin_4th_axis_leader-
         } else {
            set mom_sys_leader(fifth_axis) $mom_kin_5th_axis_leader-
         }
      } elseif { $dir == 0 } {
         if { $axis == 3 } {
            set mom_out_angle_pos($a) [ROTSET $ang $prev_angles(0) $mom_kin_4th_axis_direction\
                                                   $mom_kin_4th_axis_leader mom_sys_leader(fourth_axis)\
                                                   $mom_kin_4th_axis_min_limit $mom_kin_4th_axis_max_limit]
         } else {
            set mom_out_angle_pos($a) [ROTSET $ang $prev_angles(1) $mom_kin_5th_axis_direction\
                                                   $mom_kin_5th_axis_leader mom_sys_leader(fifth_axis)\
                                                   $mom_kin_5th_axis_min_limit $mom_kin_5th_axis_max_limit]
         }
      } elseif { $dir == 1 } {
         set mom_out_angle_pos($a) $ang
      }
   }


  #<04-25-2013 gsl> No clamp code output when rotation is ref only.
   if { ![string compare "OFF" $mom_rotation_reference_mode] } {
      global mom_sys_auto_clamp

      if { [info exists mom_sys_auto_clamp] && [string match "ON" $mom_sys_auto_clamp] } {

         set out1 "1"
         set out2 "0"

         if { $axis == 3 } { ;# Rotate 4th axis
            AUTO_CLAMP_2 $out1
            AUTO_CLAMP_1 $out2
         } else {
            AUTO_CLAMP_1 $out1
            AUTO_CLAMP_2 $out2
         }
      }
   }


   if { $axis == 3 } {

      ####  <rws>
      ####  Use ROTREF switch ON to not output the actual 4th axis move

      if { ![string compare "OFF" $mom_rotation_reference_mode] } {
         PB_CMD_fourth_axis_rotate_move
      }

      if { ![string compare "SIGN_DETERMINES_DIRECTION" $mom_kin_4th_axis_direction] } {
         set mom_prev_rot_ang_4th [expr abs($mom_out_angle_pos(0))]
      } else {
         set mom_prev_rot_ang_4th $mom_out_angle_pos(0)
      }

      MOM_reload_variable mom_prev_rot_ang_4th

   } else {

      if { [info exists mom_kin_5th_axis_direction] } {

         ####  <rws>
         ####  Use ROTREF switch ON to not output the actual 5th axis move

         if { ![string compare "OFF" $mom_rotation_reference_mode] } {
            PB_CMD_fifth_axis_rotate_move
         }

         if { ![string compare "SIGN_DETERMINES_DIRECTION" $mom_kin_5th_axis_direction] } {
            set mom_prev_rot_ang_5th [expr abs($mom_out_angle_pos(1))]
         } else {
            set mom_prev_rot_ang_5th $mom_out_angle_pos(1)
         }

         MOM_reload_variable mom_prev_rot_ang_5th
      }
   }

  #<05-10-06 sws> pb351 - Uncommented next 3 lines
  #<01-07-10 wbh> Reset mom_prev_pos using the variable mom_out_angle_pos
  # set mom_prev_pos($axis) $ang
   set mom_prev_pos($axis) $mom_out_angle_pos([expr $axis-3])
   MOM_reload_variable -a mom_prev_pos
   MOM_reload_variable -a mom_out_angle_pos
}


#=============================================================
proc PB_CMD_kin_abort_event { } {
#=============================================================
   if { [llength [info commands PB_CMD_abort_event]] } {
      PB_CMD_abort_event
   }
}


#=============================================================
proc PB_CMD_kin_before_motion { } {
#=============================================================
#  This custom command is used by UG Post to support Set/Lock,
#  rotary axis limit violation retracts and auto clamping.
#
#  --> Do not change this command!  If you want to improve
#      performance, you may comment out any of these commands.
#
#-------------------------------------------------------------
# Sep-30-2016 gsl - Allow motions to be validated against 3-axis mill post
#

   global mom_kin_machine_type

   if { [info exists mom_kin_machine_type] && [string match "*lathe*" $mom_kin_machine_type] } {
return
   }



  # Validate legitimate motion
   if { ![VALIDATE_MOTION] } {

     # PB_CMD_abort_event should be revised to handle the new abort level.
     # To abort the motion completely, it should not unset mom_sys_abort_next_event immediately.

      set ::mom_sys_abort_next_event 3
      return
   }



   if { [info exists mom_kin_machine_type] && [string match "*3_axis_mill*" $mom_kin_machine_type] } {
return
   }



  # Lock on and not circular move
   global mom_sys_lock_status  ;# Set in MOM_lock_axis
   global mom_current_motion
   if { [info exists mom_sys_lock_status] && ![string compare "ON" $mom_sys_lock_status] } {
      if { [info exists mom_current_motion] && [string compare "circular_move" $mom_current_motion] } {

         LOCK_AXIS_MOTION
      }
   }


  # Handle rotary over travel for linear moves (mom_sys_rotary_error set in PB_CMD__catch_warning)
   global mom_sys_rotary_error mom_motion_event
   if { [info exists mom_sys_rotary_error] } {
      if { $mom_sys_rotary_error != 0 && \
           [info exists mom_motion_event] && ![string compare "linear_move" $mom_motion_event] } {

         ROTARY_AXIS_RETRACT
      }

     # Error state s/b reset every time to avoid residual effect!
      UNSET_VARS mom_sys_rotary_error
   }


  # Auto clamp on
   global mom_sys_auto_clamp
   if { [info exists mom_sys_auto_clamp] && ![string compare "ON" $mom_sys_auto_clamp] } {

      AUTO_CLAMP
   }
}


#=============================================================
proc PB_CMD_kin_before_output { } {
#=============================================================
# Broker command ensuring PB_CMD_before_output,if present, gets executed
# by MOM_before_output.
#
   if [llength [info commands PB_CMD_before_output] ] {
      PB_CMD_before_output
   }
}


#=============================================================
proc PB_CMD_kin_catch_warning { } {
#=============================================================
# Called by PB_catch_warning
#
# - String with "mom_warning_info" (come from event generator or handlers)
#   may be output by MOM_catch_warning to the message file.
#
# - "mom_warning_info" will be transfered to "mom_sys_rotary_error" for
#   PB_CMD_kin_before_motion to handle the error condition.
#

  global mom_sys_rotary_error mom_warning_info

   if { [string match "ROTARY CROSSING LIMIT." $mom_warning_info] } {
      set mom_sys_rotary_error $mom_warning_info
   }

   if { [string match "secondary rotary position being used" $mom_warning_info] } {
      set mom_sys_rotary_error $mom_warning_info
   }

   if { [string match "WARNING: unable to determine valid rotary positions" $mom_warning_info] } {

     # To abort the current event
     # - Whoever handles this condition MUST unset it to avoid any lingering effect!
     #
      global mom_sys_abort_next_event
      set mom_sys_abort_next_event 1
   }
}


#=============================================================
proc PB_CMD_kin_end_of_path { } {
#=============================================================
  # Record tool time for this operation.
   if { [llength [info commands PB_CMD_set_oper_tool_time] ] } {
      PB_CMD_set_oper_tool_time
   }

  # Clear tool holder angle used in operation
   global mom_use_b_axis
   UNSET_VARS mom_use_b_axis
}


#=============================================================
proc PB_CMD_kin_feedrate_set { } {
#=============================================================
# This command supercedes the functionalites provided by the
# FEEDRATE_SET in ugpost_base.tcl.  Post Builder automatically
# generates proper call sequences to this command in the
# Event handlers.
#
# This command must be used in conjunction with ugpost_base.tcl.
#
   global   feed com_feed_rate
   global   mom_feed_rate_output_mode super_feed_mode feed_mode
   global   mom_cycle_feed_rate_mode mom_cycle_feed_rate
   global   mom_cycle_feed_rate_per_rev
   global   mom_motion_type
   global   Feed_IPM Feed_IPR Feed_MMPM Feed_MMPR Feed_INV
   global   mom_sys_feed_param
   global   mom_sys_cycle_feed_mode


   set super_feed_mode $mom_feed_rate_output_mode

   set f_pm [ASK_FEEDRATE_FPM]
   set f_pr [ASK_FEEDRATE_FPR]

   switch $mom_motion_type {

      CYCLE {
         if { [info exists mom_sys_cycle_feed_mode] } {
            if { [string compare "Auto" $mom_sys_cycle_feed_mode] } {
               set mom_cycle_feed_rate_mode $mom_sys_cycle_feed_mode
            }
         }
         if { [info exists mom_cycle_feed_rate_mode] }    { set super_feed_mode $mom_cycle_feed_rate_mode }
         if { [info exists mom_cycle_feed_rate] }         { set f_pm $mom_cycle_feed_rate }
         if { [info exists mom_cycle_feed_rate_per_rev] } { set f_pr $mom_cycle_feed_rate_per_rev }
      }

      FROM -
      RETRACT -
      RETURN -
      LIFT -
      TRAVERSAL -
      GOHOME -
      GOHOME_DEFAULT -
      RAPID {
         SUPER_FEED_MODE_SET RAPID
      }

      default {
         if { [EQ_is_zero $f_pm] && [EQ_is_zero $f_pr] } {
            SUPER_FEED_MODE_SET RAPID
         } else {
            SUPER_FEED_MODE_SET CONTOUR
         }
      }
   }


   set feed_mode $super_feed_mode


  # Adjust feedrate format per Post output unit again.
   global mom_kin_output_unit
   if { ![string compare "IN" $mom_kin_output_unit] } {
      switch $feed_mode {
         MMPM {
            set feed_mode "IPM"
            CATCH_WARNING "Feedrate mode MMPM changed to IPM"
         }
         MMPR {
            set feed_mode "IPR"
            CATCH_WARNING "Feedrate mode MMPR changed to IPR"
         }
      }
   } else {
      switch $feed_mode {
         IPM {
            set feed_mode "MMPM"
            CATCH_WARNING "Feedrate mode IPM changed to MMPM"
         }
         IPR {
            set feed_mode "MMPR"
            CATCH_WARNING "Feedrate mode IPR changed to MMPR"
         }
      }
   }


   switch $feed_mode {
      IPM     -
      MMPM    { set feed $f_pm }
      IPR     -
      MMPR    { set feed $f_pr }
      DPM     { set feed [PB_CMD_FEEDRATE_DPM] }
      FRN     -
      INVERSE { set feed [PB_CMD_FEEDRATE_NUMBER] }
      default {
         CATCH_WARNING "INVALID FEED RATE MODE"
         return
      }
   }


  # Post Builder provided format for the current mode:
   if { [info exists mom_sys_feed_param(${feed_mode},format)] } {
      MOM_set_address_format F $mom_sys_feed_param(${feed_mode},format)
   } else {
      switch $feed_mode {
         IPM     -
         MMPM    -
         IPR     -
         MMPR    -
         DPM     -
         FRN     { MOM_set_address_format F Feed_${feed_mode} }
         INVERSE { MOM_set_address_format F Feed_INV }
      }
   }

  # Commentary output
   set com_feed_rate $f_pm


  # Execute user's command, if any.
   if { [llength [info commands "PB_CMD_FEEDRATE_SET"]] } {
      PB_CMD_FEEDRATE_SET
   }
}


#=============================================================
proc PB_CMD_kin_handle_sync_event { } {
#=============================================================
   PB_CMD_handle_sync_event
}


#=============================================================
proc PB_CMD_kin_init_new_iks { } {
#=============================================================
   global mom_new_iks_exists

  # Revert legacy dual-head kinematic parameters when new IKS is absent.
   if { ![info exists mom_new_iks_exists] } {
      set ugii_version [string trim [MOM_ask_env_var UGII_VERSION]]
      if { ![string match "v3" $ugii_version] } {

         if { [llength [info commands PB_CMD_revert_dual_head_kin_vars] ] } {
            PB_CMD_revert_dual_head_kin_vars
         }
return
      }
   }

  # Initialize new IKS parameters.
   if { [llength [info commands PB_init_new_iks] ] } {
      PB_init_new_iks
   }

  # Users can provide next command to modify or disable new IKS options.
   if { [llength [info commands PB_CMD_revise_new_iks] ] } {
      PB_CMD_revise_new_iks
   }

  # Revert legacy dual-head kinematic parameters when new IKS is disabled.
   global mom_kin_iks_usage
   if { $mom_kin_iks_usage == 0 } {
      if { [llength [info commands PB_CMD_revert_dual_head_kin_vars] ] } {
         PB_CMD_revert_dual_head_kin_vars
      }
   }
}


#=============================================================
proc PB_CMD_kin_init_probing_cycles { } {
#=============================================================
   set cmd PB_CMD_init_probing_cycles
   if { [llength [info commands "$cmd"]] } {
      eval $cmd
   }
}


#=============================================================
proc PB_CMD_kin_init_rotary { } {
#=============================================================
# Following commands are defined (via uplevel) here:
#
#    MOM_clamp
#    MOM_rotate
#    MOM_lock_axis
#    PB_catch_warning
#    MOM_lintol
#

   global mom_kin_machine_type

   if { [info exists mom_kin_machine_type] } {
      if { [string match "*3_axis_mill*" $mom_kin_machine_type] ||\
           [string match "*lathe*" $mom_kin_machine_type] } {
return
      }
   }


   if { [llength [info commands PB_CMD_init_rotary] ] } {
      PB_CMD_init_rotary
   }


#***********
uplevel #0 {


#=============================================================
### This is the backup of original MOM_clamp handler.
###
### - New command PB_CMD_MOM_clamp is created with the
###   same content of the original command and executed
###   by the new MOM_clamp handler.
###
proc DUMMY_MOM_clamp { } {
#=============================================================
  global mom_clamp_axis mom_clamp_status mom_sys_auto_clamp

   if { ![string compare "AUTO" $mom_clamp_axis] } {

      if { ![string compare "ON" $mom_clamp_status] } {
         set mom_sys_auto_clamp "ON"
      } elseif { ![string compare "OFF" $mom_clamp_status] } {
         set mom_sys_auto_clamp "OFF"
      }
   } else {
      CATCH_WARNING "$mom_clamp_axis not handled in current implementation!"
   }
}


#=============================================================
### This is the backup of original MOM_rotate handler.
###
### - New command PB_CMD_MOM_rotate is created with the
###   same content of the original command and executed
###   by the new MOM_rotate handler.
###
proc DUMMY_MOM_rotate { } {
#=============================================================
# This command handles a Rotate UDE.
#
# Key parameters set in UDE -
#   mom_rotate_axis_type        :  [ AAXIS | BAXIS   | CAXIS    | HEAD | TABLE | FOURTH_AXIS | FIFTH_AXIS ]
#   mom_rotation_mode           :  [ NONE  | ATANGLE | ABSOLUTE | INCREMENTAL ]
#   mom_rotation_direction      :  [ NONE  | CLW     | CCLW ]
#   mom_rotation_angle          :  Specified angle
#   mom_rotation_reference_mode :  [ ON    | OFF ]
#
   PB_CMD_kin__MOM_rotate
}


#=============================================================
### This is the backup of original MOM_lock_axis handler.
###
### - New command PB_CMD_MOM_lock_axis is created with the
###   same content of the original command and executed
###   by the new MOM_lock_axis handler.
###
proc DUMMY_MOM_lock_axis { } {
#=============================================================
# This command handles a Lock Axis UDE.
#
# Key parameters set in UDE -
#   mom_lock_axis               :  [ XAXIS | YAXIS | ZAXIS | AAXIS | BAXIS | CAXIS | FOURTH | FIFTH | OFF ]
#   mom_lock_axis_plane         :  [ XYPLANE | YZPLANE | ZXPLANE | NONE ]
#   mom_lock_axis_value         :  Angle or coordinate value in Absolute Coordinates System
#   mom_lock_axis_value_defined :  [ 0 | 1 ]
#
# 18-Sep-2015 ljt - Reset positive_radius when lock-axis is OFF

  global mom_sys_lock_value mom_sys_lock_plane
  global mom_sys_lock_axis mom_sys_lock_status

 # Check if the rotary axis is the locked axis, it must be the 4th axis for a 4-axis machine,
 # or the 5th axis for a 5-axis machine. Otherwise, an error will be returned, or lock-axis will be turned off.
 #
 # It determines the locked axis  (axis: 0=X, 1=Y, 2=Z, 3=4th, 4=5th),
 #                   locked plane (plane: 0=YZ, 1=ZX, 2=XY), and
 #                   locked value (value: angle or coordinate that can be carried out)
 #
   set status [SET_LOCK axis plane value] ;# ON/OFF/error

   # Handle "error" condition returned from SET_LOCK
   # - Message in mom_warning_info
   if { ![string compare "error" $status] } {
      global mom_warning_info
      CATCH_WARNING $mom_warning_info

      set mom_sys_lock_status OFF
   } else {
      set mom_sys_lock_status $status
      if { ![string compare "ON" $status] } {
         set mom_sys_lock_axis $axis
         set mom_sys_lock_plane $plane
         set mom_sys_lock_value $value

         LOCK_AXIS_INITIALIZE
      } else {
         global positive_radius
         set positive_radius "0"
      }
   }
}


#=============================================================
proc PB_catch_warning { } {
#=============================================================
# This command will be called by MOM_catch_warning (ugpost_base.tcl)
# while running a multi-axis post when warning condition/message
# has been issued by the event generator of NX/Post processor.
#
   PB_CMD__catch_warning
}


#=============================================================
proc MOM_lintol { } {
#=============================================================
   PB_CMD_kin__MOM_lintol
}


} ;# uplevel
#***********

}


#=============================================================
proc PB_CMD_kin_set_csys { } {
#=============================================================
   if [llength [info commands PB_CMD_set_csys] ] {
      PB_CMD_set_csys
   }

   #<02-27-13 lili> Added following codes
   # Overload IKS params from machine model.
   PB_CMD_reload_iks_parameters

   # In case Axis Rotation has been set to "reverse"
   if { [CMD_EXIST PB_CMD_reverse_rotation_vector] } {
      PB_CMD_reverse_rotation_vector
   }

}


#=============================================================
proc PB_CMD_kin_start_of_path { } {
#=============================================================
# - For mill post -
#
#  This command is executed at the start of every operation.
#  It will verify if a new head (post) was loaded and will
#  then initialize any functionality specific to that post.
#
#  It will also restore the master Start of Program &
#  End of Program event handlers.
#
#  --> DO NOT CHANGE THIS COMMAND UNLESS YOU KNOW WHAT YOU ARE DOING.
#  --> DO NOT CALL THIS COMMAND FROM ANY OTHER CUSTOM COMMAND.
#
  global mom_sys_head_change_init_program

   if { [info exists mom_sys_head_change_init_program] } {

      PB_CMD_kin_start_of_program
      unset mom_sys_head_change_init_program


     # Load alternate units' parameters
      if [CMD_EXIST PB_load_alternate_unit_settings] {
         PB_load_alternate_unit_settings
         rename PB_load_alternate_unit_settings ""
      }


     # Execute start of head callback in new post's context.
      global CURRENT_HEAD
      if { [info exists CURRENT_HEAD] && [CMD_EXIST PB_start_of_HEAD__$CURRENT_HEAD] } {
         PB_start_of_HEAD__$CURRENT_HEAD
      }

     # Restore master start & end of program handlers
      if { [CMD_EXIST "MOM_start_of_program_save"] } {
         if { [CMD_EXIST "MOM_start_of_program"] } {
            rename MOM_start_of_program ""
         }
         rename MOM_start_of_program_save MOM_start_of_program
      }
      if { [CMD_EXIST "MOM_end_of_program_save"] } {
         if { [CMD_EXIST "MOM_end_of_program"] } {
            rename MOM_end_of_program ""
         }
         rename MOM_end_of_program_save MOM_end_of_program
      }

     # Restore master head change event handler
      if { [CMD_EXIST "MOM_head_save"] } {
         if { [CMD_EXIST "MOM_head"] } {
            rename MOM_head ""
         }
         rename MOM_head_save MOM_head
      }
   }

  # Overload IKS params from machine model.
   PB_CMD_reload_iks_parameters

  # Incase Axis Rotation has been set to "reverse"
   if { [CMD_EXIST PB_CMD_reverse_rotation_vector] } {
      PB_CMD_reverse_rotation_vector
   }

  # Initialize tool time accumulator for this operation.
   if { [CMD_EXIST PB_CMD_init_oper_tool_time] } {
      PB_CMD_init_oper_tool_time
   }

  # Force out motion G code at the start of path.
   MOM_force once G_motion
}


#=============================================================
proc PB_CMD_kin_start_of_program { } {
#=============================================================
#  This command will execute the following custom commands for
#  initialization.  They will be executed once at the start of
#  program and again each time they are loaded as a linked post.
#  After execution they will be deleted so that they are not
#  present when a different post is loaded.  You may add a call
#  to any command that you want executed when a linked post is
#  loaded.
#
#  Note when a linked post is called in, the Start of Program
#  event marker is not executed again.
#
#  DO NOT REMOVE ANY LINES FROM THIS PROCEDURE UNLESS YOU KNOW
#  WHAT YOU ARE DOING.  DO NOT CALL THIS PROCEDURE FROM ANY
#  OTHER CUSTOM COMMAND.
#
   global mom_kin_machine_type


   set command_list [list]

   if { [info exists mom_kin_machine_type] } {
      if { ![string match "*3_axis_mill*" $mom_kin_machine_type] && ![string match "*lathe*" $mom_kin_machine_type] } {

         lappend command_list  PB_CMD_kin_init_rotary
      }
   }

   lappend command_list  PB_CMD_kin_init_new_iks

   lappend command_list  PB_CMD_init_pivot_offsets
   lappend command_list  PB_CMD_init_auto_retract
   lappend command_list  PB_CMD_initialize_parallel_zw_mode
   lappend command_list  PB_CMD_init_parallel_zw_mode
   lappend command_list  PB_CMD_initialize_tool_list
   lappend command_list  PB_CMD_init_tool_list
   lappend command_list  PB_CMD_init_tape_break
   lappend command_list  PB_CMD_initialize_spindle_axis
   lappend command_list  PB_CMD_init_spindle_axis
   lappend command_list  PB_CMD_initialize_helix
   lappend command_list  PB_CMD_init_helix
   lappend command_list  PB_CMD_pq_cutcom_initialize
   lappend command_list  PB_CMD_init_pq_cutcom

   lappend command_list  PB_CMD_kin_init_probing_cycles

   lappend command_list PB_DEFINE_MACROS

   if { [info exists mom_kin_machine_type] } {
      if { [string match "*3_axis_mill_turn*" $mom_kin_machine_type] } {

          lappend command_list  PB_CMD_kin_init_mill_xzc
          lappend command_list  PB_CMD_kin_mill_xzc_init
          lappend command_list  PB_CMD_kin_init_mill_turn
          lappend command_list  PB_CMD_kin_mill_turn_initialize
      }
   }


   foreach cmd $command_list {

      if { [llength [info commands "$cmd"]] } {

         # <PB v2.0.2>
         # Old init commands for XZC/MILL_TURN posts are not executed.
         # Parameters set by these commands in the v2.0 legacy posts
         # will need to be transfered to PB_CMD_init_mill_xzc &
         # PB_CMD_init_mill_turn commands respectively.

         switch $cmd {
            "PB_CMD_kin_mill_xzc_init" -
            "PB_CMD_kin_mill_turn_initialize" {}
            default { eval $cmd }
         }
         rename $cmd ""
         proc $cmd { args } {}
      }
   }
}


#=============================================================
proc PB_CMD_license { } {
#=============================================================
global customer_server_id
global customer_server_id_length

set customer_server_id(0) "1398996 - OPUS MACH LLC"
set customer_server_id(1) "1239296 - PROLIM GLOBAL"

set customer_server_id_length [expr [string length $customer_server_id(0)]+1]

if [CHECK_LICENSE] {
   PAUSE "LICENSE ERROR"
   MOM_abort "LICENSE ERROR"
   }
}


#=============================================================
proc PB_CMD_linear_move { } {
#=============================================================
#  This procedure is used by many automatic postbuilder functions
#  to output a linear move.  Do NOT add this block to events or
#  markers.  It is a static block and it is not intended to be added
#  to events.  Do NOT change the name of this procedure.
#
#  If you need a custom command to be output with a linear move block,
#  you must place a call to the custom command either before or after
#  the MOM_do_template command.
#
#  This proc is used for:
#     simulated cycles feed moves
#     mill/turn mill linearization
#     four and five axis retract and re-engage

MOM_do_template linear_move
}


#=============================================================
proc PB_CMD_machine_time { } {
#=============================================================
global mom_machine_time mom_cutting_time
MOM_output_literal "(CUTTING TIME = [format "%5.3f" $mom_cutting_time] MINUTES, MACHINE TIME = [format "%5.3f" $mom_machine_time] MINUTES)"
}


#=============================================================
proc PB_CMD_negate_R_value { } {
#=============================================================
# This command negates the value of radius when the included angle
# of an arc is greater than 180.
#
# ==> This comamnd may be added to the Circular Move event for a post
#     of Fanuc controller when the R-style circular output format is used.
#
# 10-05-11 gsl - (pb801 IR2178985) Initial version

global mom_arc_angle
global mom_arc_radius

if [expr $mom_arc_angle > 180.0] {
   set mom_arc_radius [expr -1*$mom_arc_radius]
   }
}


#=============================================================
proc PB_CMD_nurbs_end_of_program { } {
#=============================================================
#  If you have activated NURBS output in CAM,
#  place this command @ "End of Program" event marker.

global nurbs_move_flag

if { [info exists nurbs_move_flag] } {
   OPL "G05 P0"
   }
}


#=============================================================
proc PB_CMD_nurbs_initialize { } {
#=============================================================
#  You will need to activate NURBS motion in Unigraphics CAM under
#  machine control to generate NURBS events.
#
#  This procedure is used to establish NURBS parameters.  It must be
#  placed in the "Start of Program" event marker to output nurbs.

global set mom_kin_nurbs_output_type

set mom_kin_nurbs_output_type BSPLINE

MOM_reload_kinematics
}


#=============================================================
proc PB_CMD_nurbs_move { } {
#=============================================================
#  This command can be called in NURBS Move event.
#
#  You need to activate NURBS motion in Unigraphics CAM under
#  machine control to generate nurbs events and must place
#  PB_CMD_nurbs_initialize with "Start of Program" event marker.

global mom_nurbs_order
global mom_nurbs_knot_count
global mom_nurbs_point_count

global anchor_flag
global nurbs_precision
global nurbs_move_flag
global nurbs_knot_count

if { ![info exists nurbs_move_flag] } {
   OPL "G05 P10000"

   set nurbs_move_flag 1
   }

FEEDRATE_SET

if { ![info exists anchor_flag] } {
   MOM_do_template anchor_point

   set anchor_flag 0
   }

set nurbs_knot_count 0

MOM_force once G_motion order X Y Z

while { $nurbs_knot_count < $mom_nurbs_point_count } {
      MOM_do_template nurbs

      set nurbs_knot_count [expr $nurbs_knot_count + 1]
      }

while { $nurbs_knot_count < $mom_nurbs_knot_count } {
      MOM_do_template knots

      set nurbs_knot_count [expr $nurbs_knot_count + 1]
      }
}


#=============================================================
proc PB_CMD_output_M29_to_active_rigid_tap { } {
#=============================================================
# For rigid tapping, need to output "M29 S" before the first cycle to active the rigid mode.

global mom_spindle_speed

global dpp_ge

if { $dpp_ge(cycle_hole_counter) == 1 } {
   MOM_force once M_tap S
   MOM_do_template sync_tap_invoke
   }
}


#=============================================================
proc PB_CMD_output_clamp_code { } {
#=============================================================
   global mom_sys_nc_output_mode

   if { [info exists mom_sys_nc_output_mode] && [string match "PART" $mom_sys_nc_output_mode] } {
      MOM_do_template caxis_clamp
      MOM_do_template caxis_clamp_1
   }
}


#=============================================================
proc PB_CMD_output_first_rapid_move { } {
#=============================================================
global dpp_ge
global first_rapid_move_status

if { ![info exists first_rapid_move_status] } {
   PB_CMD_define_coolant_mode

   if { [string match "3" $dpp_ge(toolpath_axis_num)] } {
      MOM_force once G_motion X Y
      MOM_do_template rapid_traverse_first
      MOM_force once G_adjust H D Z
      MOM_do_template rapid_spindle_first
      }

   if { [string match "5" $dpp_ge(toolpath_axis_num)] } {
      MOM_disable_address D H
      }

   set first_rapid_move_status "FALSE"
   }
}


#=============================================================
proc PB_CMD_output_init_position { } {
#=============================================================
   global mom_sys_nc_output_mode

   MOM_do_template caxis_unclamp
   MOM_do_template caxis_unclamp_1

   if { [info exists mom_sys_nc_output_mode] && [string match "PART" $mom_sys_nc_output_mode] } {
      MOM_output_literal "G49"
      MOM_do_template initial_move_XYFBC
   } else {
      MOM_do_template initial_move_rotation
      MOM_do_template initial_move_XY
   }
}


#=============================================================
proc PB_CMD_output_machine_mode { } {
#=============================================================
#  This command is called by the initial move & first move to determine
#  the NC output mode to be in PART (TCP) or MACHINE space.
#
#  It uses "mom_5axis_control_mode" variable (from the UDE) to set
#  the output mode "mom_sys_nc_output_mode" to be PART or MACHINE.
#
#  When the UDE is not specified, the output mode may be set according to
#  other attributes such as "mom_template_type".
#
#  ==> By default, "mom_sys_nc_output_mode" is set to "AUTO".
#      See below to enhance this function -
#
#
# Revisions:
# 04-12-12 gsl - Add description & comments
#              - Disable this function by default
#

   global mom_operation_type mom_tool_axis_type mom_template_type
   global mom_5axis_control_mode
   global mom_sys_nc_output_mode
   global mom_kin_coordinate_system_type
   global mom_kin_arc_output_mode


   set mom_sys_nc_output_mode "AUTO"


  # Set output mode according to the UDE, when it's been specified.
   if { [info exists mom_5axis_control_mode] } {

      if { [string match "TCP" $mom_5axis_control_mode] } {

         set mom_sys_nc_output_mode "PART"

      } elseif { [string match "POS" $mom_5axis_control_mode] } {

         set mom_sys_nc_output_mode "MACHINE"
      }
   }


  #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  # When UDE is not in use, you may comment out next line to
  # set the output mode to TCP for all multi-axis operations
  #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
return


   if { [string match "AUTO" $mom_sys_nc_output_mode] } {

      if { $mom_template_type == "mill_multi-axis" } {

         set mom_sys_nc_output_mode "PART"
      }
   }
}


#=============================================================
proc PB_CMD_output_next_tool { } {
#=============================================================
   global mom_next_tool_status
   global mom_next_tool_number

   if { $mom_next_tool_status == "FIRST" } { return }

   if { $mom_next_tool_number == 0 } {
      MOM_output_to_listing_device "not specify the next tool"
     # MOM_abort

      return
   }

   MOM_output_literal "T$mom_next_tool_number"
}


#=============================================================
proc PB_CMD_output_seq_number { } {
#=============================================================
   MOM_output_literal "   "
}


#=============================================================
proc PB_CMD_output_seq_number_next { } {
#=============================================================
   global mom_seqnum
   set seqnum [format "%4.0f" $mom_seqnum]
   MOM_output_literal "N[expr $seqnum-10+1]"
}


#=============================================================
proc PB_CMD_output_swiveling { } {
#=============================================================
# Output G68.2 to rotate coordinate system
#
# 05-09-2013 levi - Seperate this command from PB_CMD_output_coord_rotation

  global mom_machine_mode
  global dpp_ge

  if {[string compare $mom_machine_mode "MILL"]} {
return
  }
  if {$dpp_ge(coord_rot) == "NONE"} {
return
  }

  MOM_do_template three_plus_two_suppress CREATE
  #MOM_disable_address G_plane fourth_axis fifth_axis
  MOM_force once X Y Z

  if {$dpp_ge(sys_coord_rotation_output_type) == "SWIVELING"} {
     MOM_do_template swiveling_coord_rot
     MOM_output_literal "G53.1"
     #for {set i 0} {$i<3} {incr i} {
        #set dpp_ge(prev_coord_rotation,$i) $dpp_ge(coord_rotation,$i)
     #}
  } else {
return
  }
}


#=============================================================
proc PB_CMD_output_tcp_code { } {
#=============================================================
   global mom_sys_nc_output_mode
   global mom_sys_adjust_code

   if { $mom_sys_nc_output_mode == "PART" } {
      set mom_sys_adjust_code 43.4
     # MOM_force once G_return G_mode Z
     # MOM_do_template go_home_z
     # MOM_force once G_return G_mode X Y fourth_axis fifth_axis
     # MOM_do_template go_home_xybc
   }
}


#=============================================================
proc PB_CMD_output_unclamp_code { } {
#=============================================================
   global mom_sys_nc_output_mode

   if { [info exists mom_sys_nc_output_mode] && [string match "PART" $mom_sys_nc_output_mode] } {
      MOM_do_template caxis_unclamp
      MOM_do_template caxis_unclamp_1
   }
}


#=============================================================
proc PB_CMD_output_wcs_rotation { } {
#=============================================================
# Output G68 to rotate coordinate system
#
# 05-09-2013 levi - Seperate this command from PB_CMD_output_coord_rotation

  global mom_machine_mode
  global dpp_ge
  global mom_sys_adjust_code

  if {[string compare $mom_machine_mode "MILL"]} {
return
  }

  if {$dpp_ge(coord_rot) == "NONE"} {
return
  }

  MOM_do_template three_plus_two_suppress CREATE
  #MOM_disable_address G_plane fourth_axis fifth_axis
  MOM_force once X Y Z

  if {$dpp_ge(sys_coord_rotation_output_type) == "WCS_ROTATION"} {
     for {set i 0} {$i<3} {incr i} {
        set dpp_ge(prev_g68_first_vec,$i) $dpp_ge(g68_first_vec,$i)
        set dpp_ge(prev_g68_second_vec,$i) $dpp_ge(g68_second_vec,$i)
        set dpp_ge(prev_coord_offset,$i) $dpp_ge(coord_offset,$i)
     }
     for {set i 0} {$i<2} {incr i} {
        set dpp_ge(prev_g68_coord_rotation,$i) $dpp_ge(g68_coord_rotation,$i)
     }

     # Adjust the output for G68 if one or two of the angles are 0, under this condition just output G68 once or less,
     # otherwise should output G68 twice
     if {[EQ_is_equal $dpp_ge(g68_coord_rotation,0) 0]} {
        if {![EQ_is_equal $dpp_ge(coord_offset,0) 0]||![EQ_is_equal $dpp_ge(coord_offset,1) 0]||![EQ_is_equal $dpp_ge(coord_offset,2) 0]} {
           for {set i 0} {$i<3} {incr i} {
              set dpp_ge(g68_first_vec,$i) $dpp_ge(g68_second_vec,$i)
           }
           set dpp_ge(g68_coord_rotation,0) $dpp_ge(g68_coord_rotation,1)
           MOM_force once rotate_X rotate_Y rotate_Z rotate_i rotate_j rotate_k rotate_r
           MOM_do_template g68_first_coord_rot
        } elseif {![EQ_is_equal $dpp_ge(g68_coord_rotation,1) 0]} {
           MOM_force once rotate_X rotate_Y rotate_Z rotate_i rotate_j rotate_k rotate_r
           MOM_do_template g68_second_coord_rot
        }
     } elseif {[EQ_is_equal $dpp_ge(g68_coord_rotation,1) 0]} {
        MOM_force once rotate_X rotate_Y rotate_Z rotate_i rotate_j rotate_k rotate_r
        MOM_do_template g68_first_coord_rot
     } else {
        MOM_force once rotate_X rotate_Y rotate_Z rotate_i rotate_j rotate_k rotate_r
        MOM_do_template g68_first_coord_rot
        MOM_force once rotate_X rotate_Y rotate_Z rotate_i rotate_j rotate_k rotate_r
        MOM_do_template g68_second_coord_rot
     }
  } else {
return
  }
}


#=============================================================
proc PB_CMD_patch_fixture_offset_blocks { } {
#=============================================================
# This command will be called by PB_CMD_fixture_offset
# to add the definition of block templates needed
# to output fixture offset instructions.
#
   global mom_logname

   if { [MOM_has_definition_element BLOCK fixture_number] &&\
        [MOM_has_definition_element BLOCK fixture_number_enhancement] &&\
        [MOM_has_definition_element ADDRESS P_fixture_offset] } {

      # Both blocks & needed address exist, do nothing -
return
   }

   set ugii_tmp_dir [MOM_ask_env_var UGII_TMP_DIR]
   set def_file_name  [file join $ugii_tmp_dir __${mom_logname}_patch_fixture_offset_blocks_[clock clicks].def]

   if { [catch { set tmp_file [open "$def_file_name" w] } res] } {

      if { ![info exists res] } {
         set res "$def_file_name\nFile open error!"
      }

      if { [llength [info commands PAUSE] ] } {
         PAUSE "Definition patch file error" "$res"
      }

      CATCH_WARNING "$res"
return
   }


   fconfigure $tmp_file -translation lf

   puts $tmp_file "MACHINE mill"
   puts $tmp_file "FORMATTING"
   puts $tmp_file "{"

   if { ![MOM_has_definition_element ADDRESS P_fixture_offset] } {
      puts $tmp_file "  ADDRESS P_fixture_offset"
      puts $tmp_file "  {"
      puts $tmp_file "      FORMAT      Digit_2"
      puts $tmp_file "      FORCE       off"
      puts $tmp_file "      MAX         99 Truncate"
      puts $tmp_file "      MIN         1 Truncate"
      puts $tmp_file "      LEADER      \"P\""
      puts $tmp_file "  }"
   }

   if { ![MOM_has_definition_element BLOCK fixture_number] } {
      puts $tmp_file "  BLOCK_TEMPLATE fixture_number"
      puts $tmp_file "  {"
      puts $tmp_file "       G\[\$mom_fixture_offset_value + 53\]"
      puts $tmp_file "  }"
   }

   if { ![MOM_has_definition_element BLOCK fixture_number_enhancement] } {
      puts $tmp_file "  BLOCK_TEMPLATE fixture_number_enhancement"
      puts $tmp_file "  {"
      puts $tmp_file "       Text\[G54.1\]"
      puts $tmp_file "       P_fixture_offset\[\$mom_fixture_offset_value\]"
      puts $tmp_file "  }"
   }

   puts $tmp_file "}"

   close $tmp_file

   global tcl_platform
   if { [string match "*windows*" $tcl_platform(platform)] } {
      regsub -all {/} $def_file_name {\\} def_file_name
   }

   if { [catch { MOM_load_definition_file  "$def_file_name" } res] } {
      CATCH_WARNING $res
   }

   MOM_remove_file $def_file_name
}


#=============================================================
proc PB_CMD_pause { } {
#=============================================================
# This command enables you to pause the UG/Post processing.

PAUSE
}


#=============================================================
proc PB_CMD_position_tool_to_R_point_with_no_clearance_plane { } {
#=============================================================
# For first move won't call MOM_rapid_move, need to position tool to R point
# before calling cycle under "AUTO_3D" and "LOCAL" condition if there is no clearance
# plane or start point.

global mom_motion_type
global mom_current_motion

global dpp_ge

if { [info exists mom_motion_type] && $mom_motion_type == "CYCLE" && [string compare $dpp_ge(coord_rot)  "NONE"] && \
     [info exists mom_current_motion] && [string match "first_move" $mom_current_motion] } {
   MOM_force once G_adjust H
   MOM_rapid_move
   }
}


#=============================================================
proc PB_CMD_program_header { } {
#=============================================================
#
#  Program Header with Tape Number
#
#  This procedure will output a program header with the following format:
#
#  Attribute assigned to program (Name of program group)
#  O0001 (NC_PROGRAM)
#
#  Place this custom command in the start of program event marker.  This
#  custom command must be placed after any intial codes (such as #).  The
#  custom comand MOM_set_seq_off must precede this custom command to
#  prevent sequence numbers from being output with the program number.
#
#  If you are adding this custom command to a linked post, this custom
#  command must be added to the main post only.  It will not be output by
#  any subordinate posts.
#
#  If there is no attribute assigned to the program group, the string O0001
#  will be used.  In any case the name of the program in Program View will
#  be output as a comment.
#
#  To assign an attribute to the program, right click on the program.  Under
#  properties, select attribute.  Use the string "program_number" as the
#  title of the attribute.  Enter the string you need for the program
#  name, O0010 for example, as the value of the attribute.  Use type string for the
#  the attribute.  Each program group can have a unique program number.
#
   global mom_attr_PROGRAMVIEW_PROGRAM_NUMBER
   global program_header_output mom_oper_program
   global debug mom_event_handler_file_name
   set debug 0
   if [info exists program_header_output] { return }

   set program_header_output 1

global mom_output_file_basename
global mom_logname mom_part_name
global mom_group_name mom_parent_group_name
global mom_date
global mom_output_unit mom_output_file_basename

## Get the Post Name
set post_name [file tail $mom_event_handler_file_name]
set post_name_length [string length $post_name]
set post_name [string range $post_name 0 [expr ($post_name_length - 5)]]
#Set date and time
set file_time [string range $mom_date 11 15]

set output_group_name "99999"
if { [info exists mom_parent_group_name] } {
     set output_group_name $mom_parent_group_name
   } elseif { [info exists mom_group_name] } {
     set output_group_name $mom_group_name
   } else {
     set output_group_name $mom_oper_program
   }
set output_group_name $mom_output_file_basename
#switch $mom_run_number {
#                       1 {MOM_output_literal "$mom_attr_PROGRAMVIEW_PROGRAM_1"}
#                       2 {MOM_output_literal "$mom_attr_PROGRAMVIEW_PROGRAM_2"}
#                       }
set part_name [file tail $mom_part_name]
set PH  "NA"
set PRH "NA"
set PN  "NA"
set PRN "NA"
set PNAME "NA"
##CHeck if part name is from TCE
if { [string match "*UGMGR=*" $part_name] } {
      set name_list [split $part_name " "]
      foreach V1 $name_list {
          if { [string match "PH=*" $V1] } {
                 set a [string length $V1]
                 set a [expr ($a-1)]
                 set PH [string range $V1 3 $a]
             }
          if { [string match "PRH=*" $V1] } {
                 set a [string length $V1]
                 set a [expr ($a-1)]
                 set PRH [string range $V1 4 $a]
             }
          if { [string match "PN=*" $V1] } {
                 set a [string length $V1]
                 set a [expr ($a-1)]
                 set PN [string range $V1 3 $a]
             }
          if { [string match "PRN=*" $V1] } {
                 set a [string length $V1]
                 set a [expr ($a-1)]
                 set PRN [string range $V1 4 $a]
             }
      }
   }
if { $PN != "NA" } {
        set part_name $PN
   } else {
        set part_name [format "%18s" $part_name]
   }

###Use Part Attributes
global mom_attr_PART_DB_PART_MFKID mom_attr_PART_DB_PART_NAME

##Part ID
if { [info exists mom_attr_PART_DB_PART_MFKID] } {
     set PN $mom_attr_PART_DB_PART_MFKID
   }
##Part Name
if { [info exists mom_attr_PART_DB_PART_NAME] } {
     set PNAME $mom_attr_PART_DB_PART_NAME
   }

##################################################################
if { $PN != "NA" } {
        set part_name $PN
   } else {
        set part_name [format "%18s" $part_name]
   }
set user_name $mom_logname

#MOM_set_seq_on
if { [string match "O*" [string toupper $output_group_name]] } {
    MOM_output_literal "[string toupper $output_group_name]"
} else {
    MOM_output_literal "O$output_group_name"
}
MOM_set_seq_off
MOM_output_literal "(PART  FILE  NAME: [string toupper $part_name] )"
MOM_output_literal "(PROGRAMMED  UNIT: $mom_output_unit)"
MOM_output_literal "(PROGRAMMED    BY: [string toupper $user_name])"
MOM_output_literal "(PROGRAMMED    ON: [ string toupper  [string range $mom_date 0 9],[string range $mom_date 20 23]],$file_time           )"
MOM_output_literal "(NX POSTPROCESSOR: [string toupper $post_name] )"
MOM_output_literal " "
}


#=============================================================
proc PB_CMD_recalculate_drilling_parameters_under_auto3d_condition { } {
#=============================================================
# Recalculate drilling parameters under auto3d condition
#
# 05-09-2013 levi - Seperate this command from PB_CMD_output_coord_rotation

global mom_pos
global mom_machine_mode
global mom_cycle_feed_to
global mom_cycle_rapid_to
global mom_cycle_retract_to
global mom_cycle_feed_to_pos
global mom_cycle_rapid_to_pos
global mom_cycle_retract_to_pos

global dpp_ge

if { [string compare "MILL" $mom_machine_mode] } {
   return
   }

if { $dpp_ge(coord_rot) == "AUTO_3D" && [DPP_GE_DETECT_HOLE_CUTTING_OPERATION] } {
   VMOV 3 mom_pos mom_cycle_rapid_to_pos
   VMOV 3 mom_pos mom_cycle_feed_to_pos
   VMOV 3 mom_pos mom_cycle_retract_to_pos

   set mom_cycle_rapid_to_pos(2)   [expr $mom_pos(2)+$mom_cycle_rapid_to]
   set mom_cycle_retract_to_pos(2) [expr $mom_pos(2)+$mom_cycle_retract_to]
   set mom_cycle_feed_to_pos(2)    [expr $mom_pos(2)+$mom_cycle_feed_to]
   }
}


#=============================================================
proc PB_CMD_recalculate_initial_pos_with_no_clearance_plane_for_cycle { } {
#=============================================================
# if the cycle operation has no clearance plane or start point, recalculate
# the initial Z position
#
# 07-23-2012 levi - Initial version

global mom_pos
global mom_motion_type
global mom_current_motion
global mom_cycle_rapid_to_pos

global dpp_ge

if { [info exists mom_motion_type] && $mom_motion_type == "CYCLE" && [string match $dpp_ge(coord_rot) "AUTO_3D"] } {
   if { [info exists mom_current_motion] } {
      if { [string match "initial_move" $mom_current_motion] || [string match "first_move" $mom_current_motion] } {
         set mom_pos(2) $mom_cycle_rapid_to_pos(2)
         }
      }
   }
}


#=============================================================
proc PB_CMD_reload_iks_parameters { } {
#=============================================================
# This command overloads new IKS params from a machine model.(NX4)
# It will be executed automatically at the start of each path
# or when CSYS has changed.

global mom_csys_matrix
global mom_kin_iks_usage

#----------------------------------------------------------
# Set a classification to fetch kinematic parameters from
# a particular set of K-components of a machine.
# - Default is NONE.
#----------------------------------------------------------
set custom_classification NONE

if { [info exists mom_kin_iks_usage] && $mom_kin_iks_usage == 1 } {
   if [info exists mom_csys_matrix] {
      if [llength [info commands MOM_validate_machine_model] ] {
         if { [MOM_validate_machine_model] == "TRUE" } {
            MOM_reload_iks_parameters "$custom_classification"
            MOM_reload_kinematics
            }
         }
      }
   }
}


#=============================================================
proc PB_CMD_remove_M29 { } {
#=============================================================
global pop_output_M29
   set pop_output_M29 0
}


#=============================================================
proc PB_CMD_remove_Q0 { } {
#=============================================================
global mom_cycle_step1

if { [EQ_is_zero $mom_cycle_step1] } {
   MOM_suppress once cycle_step
   }
}


#=============================================================
proc PB_CMD_remove_q0 { } {
#=============================================================
   global mom_cycle_step1

   if { [EQ_is_zero $mom_cycle_step1] } {
      MOM_suppress once cycle_step
   }
}


#=============================================================
proc PB_CMD_reposition_move { } {
#=============================================================
#  This procedure is used by rotary axis retract to reposition the
#  rotary axes after the tool has been fully retracted.
#
#  You can modify the this procedure to customize the reposition move.
#  If you need a custom command to be output with this block,
#  you must place a call a the custom command either before or after
#  the MOM_do_template command.

MOM_suppress once X Y Z
MOM_do_template rapid_traverse
}


#=============================================================
proc PB_CMD_reset_all_motion_variables_to_zero { } {
#=============================================================
# Stage for MOM_reload_kinematics

global mom_pos
global mom_prev_pos
global mom_out_angle_pos

set mom_pos(0) 0.0
set mom_pos(1) 0.0
set mom_pos(2) 0.0
set mom_pos(3) 0.0
set mom_pos(4) 0.0

set mom_prev_pos(0) 0.0
set mom_prev_pos(1) 0.0
set mom_prev_pos(2) 0.0

set mom_out_angle_pos(0) 0.0
set mom_out_angle_pos(1) 0.0

MOM_reload_variable -a mom_prev_pos
MOM_reload_variable -a mom_pos
MOM_reload_variable -a mom_out_angle_pos

MOM_reload_kinematics
}


#=============================================================
proc PB_CMD_reset_auto_detected_parameter { } {
#=============================================================
# Reset the dpp values which are automatically detected in post processor, including
# tool path type, coordinate system rotation type, hole counter and previous parameters
# of 3+2 machining. Please put this command in start of path and don't move to other place.

RESET_DPP_VALUE
}


#=============================================================
proc PB_CMD_reset_force_cycle_parameters { } {
#=============================================================
global force_cycle_parameters

if { [info exists force_cycle_parameters] } { unset force_cycle_parameters }
}


#=============================================================
proc PB_CMD_reset_output_mode { } {
#=============================================================
# Reset tool path type and output type
# Used in end of path

global mom_sys_adjust_code
global mom_next_oper_has_tool_change
global mom_current_oper_is_last_oper_in_program

global dpp_ge
global first_tool_status

MOM_enable_address G_adjust D H

# Cancel coordinate system rotation G254 command.
if { [string match "YES" $mom_next_oper_has_tool_change] || [string match "YES" $mom_current_oper_is_last_oper_in_program] } {
   if { [string compare "NONE" $dpp_ge(coord_rot)] && [string match "3" $dpp_ge(toolpath_axis_num)] } {
      MOM_force once G_dwo
      MOM_do_template cancel_dynamic_work_offset
      }
   }

# Cancel Tool Center Point Control command
if { [string match "5" $dpp_ge(toolpath_axis_num)] } {
       #MOM_force once G_adjust
       #MOM_do_template tool_len_adj_off
      MOM_force once G_dwo
      MOM_do_template cancel_dynamic_work_offset
   }

# Reset tool length compensation code.
set mom_sys_adjust_code 43

# Restore kinematics to original kinematics.
DPP_GE_RESTORE_KINEMATICS

set first_tool_status "FALSE"
}


#=============================================================
proc PB_CMD_restore_active_oper_tool_data { } {
#=============================================================
#  This command restores the attributes of the tool used in the current operation
#  to be post-processed before the generation of the tool list.
#  The attributes have been saved in proc PB_CMD_save_active_oper_tool_data.
#  This command wil be executed automatically in PB_CMD_create_tool_list.

global mom_sys_oper_tool_attr_list
global mom_sys_oper_tool_attr_saved_arr

foreach mom_var $mom_sys_oper_tool_attr_list {
        global $mom_var

        if [info exists mom_sys_oper_tool_attr_saved_arr($mom_var)] {
           set $mom_var $mom_sys_oper_tool_attr_saved_arr($mom_var)
           }
        }
}


#=============================================================
proc PB_CMD_restore_kinematics { } {
#=============================================================
# Restore original kinematics variables

set kin_list { mom_kin_arc_output_mode mom_kin_helical_arc_output_mode mom_sys_4th_axis_has_limits mom_sys_5th_axis_has_limits mom_kin_machine_type mom_kin_4th_axis_ang_offset mom_kin_arc_output_mode mom_kin_4th_axis_direction mom_kin_4th_axis_incr_switch mom_kin_4th_axis_leader mom_kin_4th_axis_limit_action mom_kin_4th_axis_max_limit mom_kin_4th_axis_min_incr mom_kin_4th_axis_min_limit mom_kin_4th_axis_plane mom_kin_4th_axis_rotation mom_kin_4th_axis_type mom_kin_5th_axis_zero mom_kin_4th_axis_zero mom_kin_5th_axis_direction mom_kin_5th_axis_incr_switch mom_kin_5th_axis_leader mom_kin_5th_axis_limit_action mom_kin_5th_axis_max_limit mom_kin_5th_axis_min_incr mom_kin_5th_axis_min_limit mom_kin_5th_axis_plane mom_kin_5th_axis_rotation mom_kin_5th_axis_type mom_kin_5th_axis_ang_offset }
set kin_array_list { mom_kin_spindle_axis mom_kin_4th_axis_center_offset mom_kin_5th_axis_center_offset mom_kin_4th_axis_point mom_kin_5th_axis_point mom_kin_4th_axis_vector mom_kin_5th_axis_vector }

foreach kin_var $kin_list {
        global $kin_var
        global save_$kin_var

        if { [info exists save_$kin_var] } {
           set value [set save_$kin_var]
           set $kin_var $value

           unset save_$kin_var
           }
        }

foreach kin_var $kin_array_list {
        global $kin_var
        global save_$kin_var

        if { [array exists save_$kin_var] } {
           set save_var save_$kin_var

           VMOV 3 $save_var $kin_var

           UNSET_VARS $save_var
           }
        }

global mom_sys_leader

if { [info exists mom_kin_4th_axis_leader] && [info exists mom_kin_5th_axis_leader] } {
   set mom_sys_leader(fourth_axis) $mom_kin_4th_axis_leader
   set mom_sys_leader(fifth_axis) $mom_kin_5th_axis_leader
   }

MOM_reload_kinematics
}


#=============================================================
proc PB_CMD_restore_work_plane_change { } {
#=============================================================
#Restore work plane change flag, if being disabled due to a simulated cycle.

global mom_user_spindle_first
global mom_sys_work_plane_change
global mom_user_work_plane_change

global spindle_first

if { [info exists mom_user_work_plane_change] } {
   set mom_sys_work_plane_change $mom_user_work_plane_change
   set spindle_first $mom_user_spindle_first

   unset mom_user_work_plane_change
   unset mom_user_spindle_first
   }
}


#=============================================================
proc PB_CMD_retract_move { } {
#=============================================================
#  This procedure is used by rotary axis retract to move away from
#  the part.  This move is a three axis move along the tool axis at
#  a retract feedrate.
#
#  You can modify the this procedure to customize the retract move.
#  If you need a custom command to be output with this block,
#  you must place a call to the custom command either before or after
#  the MOM_do_template command.
#
#  If you need to modify the x,y or z locations you will need to do the
#  following.  (without the #)
#
#  global mom_pos
#  set mom_pos(0) 1.0
#  set mom_pos(1) 2.0
#  set mom_pos(2) 3.0

MOM_do_template linear_move
}


#=============================================================
proc PB_CMD_return_home { } {
#=============================================================

}


#=============================================================
proc PB_CMD_return_to_reference_point { } {
#=============================================================
# If next operation has tool change or it's the last operation, go to reference point. Otherwise,
# don't go to reference point.
#
# 06-07-2013 levi - Initial version.

  global mom_next_oper_has_tool_change
  global mom_next_tool_status

  if {$mom_next_oper_has_tool_change == "YES" || $mom_next_tool_status == "FIRST" } {
     MOM_do_template return_to_reference_Z
     MOM_do_template return_to_reference_XY
     MOM_do_template return_rotary_axis_to_zero
     MOM_do_template spindle_off
     MOM_do_template coolant_off
  }


}


#=============================================================
proc PB_CMD_reverse_rotation_vector { } {
#=============================================================
# This command fixes the vectors of rotary axes.
# It will be executed automatically when present.
# Do not attach it with any Event markers.

global mom_csys_matrix
global mom_kin_iks_usage

set reverse_vector 0

if { [info exists mom_kin_iks_usage] && $mom_kin_iks_usage == 1 } {
   if { [info exists mom_csys_matrix] } {
      if { [llength [ info commands MOM_validate_machine_model ]] } {
         if { ![string compare "TRUE" [ MOM_validate_machine_model ]] } {
            set reverse_vector 1
            }
         }
      }
   }

if $reverse_vector {
   global mom_kin_4th_axis_vector
   global mom_kin_5th_axis_vector
   global mom_kin_4th_axis_rotation
   global mom_kin_5th_axis_rotation

   foreach axis { 4th_axis 5th_axis } {
           if { [info exists mom_kin_${axis}_rotation] && [string match "reverse" [set mom_kin_${axis}_rotation]] } {
              if { [info exists mom_kin_${axis}_vector] } {
                 foreach i { 0 1 2 } {
                         set mom_kin_${axis}_vector($i) [expr -1 * [set mom_kin_${axis}_vector($i)]]
                         }
                 }
              }
           }

   MOM_reload_kinematics
   }
}


#=============================================================
proc PB_CMD_revert_dual_head_kin_vars { } {
#=============================================================
# Only dual-head 5-axis mill posts will be affected by this
# command.
#
# This command reverts kinematic parameters for dual-head 5-axis
# mill posts to maintain compatibility and to allow the posts
# to run in UG/Post prior to NX3.
#
# Attributes of the 4th & 5th Addresses, their locations in
# the Master Word Sequence and all the Blocks that use these
# Addresses will be reconditioned with call to
#
#     PB_swap_dual_head_elements
#
#-------------------------------------------------------------
# 04-15-05 gsl - Added for PB v3.4
#-------------------------------------------------------------

global mom_kin_machine_type

if { ![string match  "5_axis_dual_head"  $mom_kin_machine_type] } {
   return
   }

set var_list { ang_offset center_offset(0) center_offset(1) center_offset(2) direction incr_switch leader limit_action max_limit min_incr min_limit plane rotation zero }

set center_offset_set 0

foreach var $var_list {
        # Global declaration
        if { [string match "center_offset*" $var] } {
           if { !$center_offset_set } {
              global mom_kin_4th_axis_center_offset
              global mom_kin_5th_axis_center_offset

              set center_offset_set 1
              }

           } else {
                  global mom_kin_4th_axis_[set var]
                  global mom_kin_5th_axis_[set var]
                  }

        # Swap values
        set val [set mom_kin_4th_axis_[set var]]
        set mom_kin_4th_axis_[set var] [set mom_kin_5th_axis_[set var]]
        set mom_kin_5th_axis_[set var] $val
        }

# Update kinematic parameters
MOM_reload_kinematics

# Swap address leaders
global mom_sys_leader

set val $mom_sys_leader(fourth_axis)
set mom_sys_leader(fourth_axis) $mom_sys_leader(fifth_axis)
set mom_sys_leader(fifth_axis)  $val

# Swap elements in definition file
if { [llength [info commands PB_swap_dual_head_elements] ] } {
   PB_swap_dual_head_elements
   }
}


#=============================================================
proc PB_CMD_revise_new_iks { } {
#=============================================================
# This command is executed automatically, which allows you
# to change the default IKS parameters or disable the IKS
# service completely.
#
# *** Do not attach this command to any event marker! ***

global mom_kin_iks_usage
global mom_kin_spindle_axis
global mom_kin_4th_axis_vector
global mom_kin_5th_axis_vector
global mom_kin_rotary_axis_method

# Uncomment next statement to disable new IKS service
# set mom_kin_iks_usage 0

# Uncomment next statement to change rotary solution method
# set mom_kin_rotary_axis_method "ZERO"

# Uncomment next statement, if any parameter above has changed.
# MOM_reload_kinematics
}


#=============================================================
proc PB_CMD_run_postprocess { } {
#=============================================================
# This is an example showing how MOM_run_postprocess can be used.
# It can be called in the Start of Program event (or anywhere)
# to process the same objects being posted with a secondary post.
#
# ==> It's advisable NOT to use the active post and the same
#     output file for this secondary posting process.
# ==> Ensure legitimate and fully qualified post processor and
#     output file are specified with the command.

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# CAUTION - Uncomment next line to activate this function!
return
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++

MOM_run_postprocess "[file dirname $::mom_event_handler_file_name]/MORI_HORI_Sub.tcl"\
                    "[file dirname $::mom_event_handler_file_name]/MORI_HORI_Sub.def"\
                    "${::mom_output_file_directory}sub_program.out"
}


#=============================================================
proc PB_CMD_save_active_oper_tool_data { } {
#=============================================================
#  This command saves the attributes of the tool used in the current operation
#  to be post-processed before the generation of the tool list.
#
#  This command will be executed automatically in PB_CMD_create_tool_list.
#
#  You may add any desired MOM variable to the list below to be restored
#  later in your post.

global mom_sys_oper_tool_attr_list
global mom_sys_oper_tool_attr_saved_arr

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# You may add any MOM variable that needs to be retained for
# the operation to the list below (using lappend command).
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
set mom_sys_oper_tool_attr_list [list]

lappend mom_sys_oper_tool_attr_list mom_tool_number
lappend mom_sys_oper_tool_attr_list mom_tool_length_adjust_register
lappend mom_sys_oper_tool_attr_list mom_tool_name
lappend mom_sys_oper_tool_attr_list mom_operation_name

foreach mom_var $mom_sys_oper_tool_attr_list {
        global $mom_var

        if [info exists $mom_var] {
           set mom_sys_oper_tool_attr_saved_arr($mom_var) [eval format %s $$mom_var]
           }
        }
}


#=============================================================
proc PB_CMD_save_kinematics { } {
#=============================================================
#This proc is used to save original kinematics variables

set kin_list { mom_sys_4th_axis_has_limits   mom_sys_5th_axis_has_limits  mom_kin_machine_type \
               mom_kin_4th_axis_ang_offset   mom_kin_arc_output_mode      mom_kin_4th_axis_direction \
               mom_kin_4th_axis_incr_switch  mom_kin_4th_axis_leader      mom_kin_4th_axis_limit_action \
               mom_kin_4th_axis_max_limit    mom_kin_4th_axis_min_incr    mom_kin_4th_axis_min_limit \
               mom_kin_4th_axis_plane        mom_kin_4th_axis_rotation    mom_kin_4th_axis_type \
               mom_kin_5th_axis_zero         mom_kin_4th_axis_zero        mom_kin_5th_axis_direction \
               mom_kin_5th_axis_incr_switch  mom_kin_5th_axis_leader      mom_kin_5th_axis_limit_action \
               mom_kin_5th_axis_max_limit    mom_kin_5th_axis_min_incr    mom_kin_5th_axis_min_limit \
               mom_kin_5th_axis_plane        mom_kin_5th_axis_rotation    mom_kin_5th_axis_type \
               mom_kin_5th_axis_ang_offset   }

set kin_array_list { mom_kin_4th_axis_center_offset  mom_kin_5th_axis_center_offset   mom_kin_4th_axis_point \
                     mom_kin_5th_axis_point          mom_kin_4th_axis_vector          mom_kin_5th_axis_vector }

foreach kin_var $kin_list {
        global $kin_var
        global save_$kin_var

        if { [info exists $kin_var] && ![info exists save_$kin_var] } {
           set value [set $kin_var]
           set save_$kin_var $value
           }
        }

foreach kin_var $kin_array_list {
        global $kin_var
        global save_$kin_var

        if { [array exists $kin_var] && ![array exists save_$kin_var] } {
           set save_var save_$kin_var

           VMOV 3 $kin_var $save_var
           }
        }

global mom_kin_read_ahead_next_motion

set mom_kin_read_ahead_next_motion "1"

MOM_reload_kinematics
}


#=============================================================
proc PB_CMD_save_last_z_for_drilling_operations { } {
#=============================================================
global mom_pos
global mom_operation_type

global save_mom_pos

if { [string match "Point to Point" $mom_operation_type] || [string match "Drilling" $mom_operation_type] || [string match "Hole Making" $mom_operation_type] } {
   set save_mom_pos(2) $mom_pos(2)
   }
}


#=============================================================
proc PB_CMD_select_mcs { } {
#=============================================================
   global mom_fixture_offset_value
   global mcs_additional_p

   if { $mom_fixture_offset_value > 6 } {
      set mcs_additional_p [expr $mom_fixture_offset_value - 6 ]
      MOM_force ONCE mcs_additional_p mcs_additional_g
      MOM_do_template output_mcs_additional
   } else {
      MOM_force ONCE G
      MOM_do_template output_mcs
   }
}


#=============================================================
proc PB_CMD_set_csys { } {
#=============================================================
# This custom command is provided as the default to nullify
# the same command in a linked post that may have been
# imported from pb_cmd_coordinate_system_rotation.tcl.
}


#=============================================================
proc PB_CMD_set_cycle_plane { } {
#=============================================================
# Use this command to determine and output proper plane code
# when G17/18/19 is used in the cycle definition.
#
# <04-15-08 gsl> - Add initialization for protection
# <03-06-08 gsl> - Declare needed global variables
# <02-28-08 gsl> - Make use of mom_spindle_axis
# <06-22-09 gsl> - Call PB_CMD_set_principal_axis
# <06-07-13 levi> - Add "MOM_suppress once G_plane" when using 3+2 mode, for
# even set working plane as XY here, it may output G18/G19 in cycle.

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This option can be set to 1, if the address of cycle's
# principal axis needs to be suppressed. (Ex. Siemens controller)
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
set suppress_principal_axis 0

#++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This option can be set to 1, if the plane code needs
# to be forced out @ the start of every set of cycles.
#++++++++++++++++++++++++++++++++++++++++++++++++++++++
set force_plane_code 0

global mom_cycle_spindle_axis

global dpp_ge

PB_CMD_set_principal_axis

#if used 3+2 mode, the working plane should be XY
if { [string compare "NONE" $dpp_ge(coord_rot)] } {
   set mom_cycle_spindle_axis 2
   set mom_cutcom_plane  XY
   set mom_pos_arc_plane XY

   MOM_suppress once G_plane
   }

switch $mom_cycle_spindle_axis {
                               0 {
                                 set principal_axis X
                                 }
                               1 {
                                 set principal_axis Y
                                 }
                               2 {
                                 set principal_axis Z
                                 }
                         default {
                                 set principal_axis ""
                                 }
                               }

if { $suppress_principal_axis && [string length $principal_axis] > 0 } {
   MOM_suppress once $principal_axis
   }

if { $force_plane_code } {
   global cycle_init_flag

   if { [info exists cycle_init_flag] && [string match "TRUE" $cycle_init_flag] } {
      MOM_force once G_plane
      }
   }

incr dpp_ge(cycle_hole_counter)
}


#=============================================================
proc PB_CMD_set_principal_axis { } {
#=============================================================
# This command can be used to determine the principal axis.
#
# => It can be used to determine a proper work plane when the
#    "Work Plane" parameter is not specified with an operation.
#
#
# <06-22-09 gsl> - Extracted from PB_CMD_set_cycle_plane
# <10-09-09 gsl> - Do not define mom_pos_arc_plane unless it doesn't exist.
# <03-10-10 gsl> - Respect tool axis for 3-axis & XZC cases
# <01-21-11 gsl> - Enhance header description

global mom_spindle_axis
global mom_cutcom_plane
global mom_pos_arc_plane
global mom_cycle_spindle_axis

# Initialization spindle axis
global mom_kin_spindle_axis
global mom_sys_spindle_axis

if { ![info exists mom_kin_spindle_axis] } {
   set mom_kin_spindle_axis(0) 0.0
   set mom_kin_spindle_axis(1) 0.0
   set mom_kin_spindle_axis(2) 1.0
   }

if { ![info exists mom_sys_spindle_axis] } {
   VMOV 3 mom_kin_spindle_axis mom_sys_spindle_axis
   }

if { ![info exists mom_spindle_axis] } {
   VMOV 3 mom_sys_spindle_axis mom_spindle_axis
   }

# Default cycle spindle axis to Z
set mom_cycle_spindle_axis 2

#<03-10-10 gsl> pb751 - Respect tool axis for 3-axis & XZC
global mom_kin_machine_type mom_tool_axis

if [string match "3_axis*" $mom_kin_machine_type] {
   VMOV 3 mom_tool_axis spindle_axis

   } else {
          VMOV 3 mom_spindle_axis spindle_axis
          }

if { [EQ_is_equal [expr abs($spindle_axis(0))] 1.0] } {
   set mom_cycle_spindle_axis 0
   }

if { [EQ_is_equal [expr abs($spindle_axis(1))] 1.0] } {
   set mom_cycle_spindle_axis 1
   }

switch $mom_cycle_spindle_axis {
                               0 {
                                 set mom_cutcom_plane  YZ
                                 }
                               1 {
                                 set mom_cutcom_plane  ZX
                                 }
                               2 {
                                 set mom_cutcom_plane  XY
                                 }
                         default {
                                 set mom_cutcom_plane  UNDEFINED
                                 }
                               }

# Set arc plane when it's not defined
if { ![info exists mom_pos_arc_plane] || $mom_pos_arc_plane == "" } {
   set mom_pos_arc_plane $mom_cutcom_plane
   }
}


#=============================================================
proc PB_CMD_set_tcp_code { } {
#=============================================================
# Command to set TCP mode.
# Used in initial move and first move

global mom_pos
global mom_mcs_goto
global mom_prev_pos
global mom_arc_center
global mom_machine_mode
global mom_prev_mcs_goto
global mom_pos_arc_center
global mom_sys_adjust_code

global dpp_ge

## return if NOT need output TCP code
if { [string compare $mom_machine_mode "MILL"] } {
   return
   }

if { [string match $dpp_ge(sys_tcp_tool_axis_output_mode) "AXIS"] && $dpp_ge(toolpath_axis_num) == "5" } {
   set mom_sys_adjust_code 43

   } elseif { [string match $dpp_ge(sys_tcp_tool_axis_output_mode) "VECTOR"] && $dpp_ge(toolpath_axis_num) == "5" } {
            set mom_sys_adjust_code 43.5
            set dpp_ge(sys_output_coord_mode) "TCP_FIX_TABLE"

            } else {
                   set mom_sys_adjust_code 43
                   }

if { [string match $dpp_ge(sys_output_coord_mode) "TCP_FIX_TABLE"] && $dpp_ge(toolpath_axis_num) == "5" } {
   VMOV 3 mom_mcs_goto mom_pos
   VMOV 3 mom_prev_mcs_goto mom_prev_pos
   VMOV 3 mom_arc_center mom_pos_arc_center
   }
}


#=============================================================
proc PB_CMD_set_working_plane { } {
#=============================================================
# Always set working plane as XY for 3+2 axis machining and 5 axis machining
#
# 05-15-2013 levi - Initial version

  global dpp_ge
  global mom_cycle_spindle_axis
  global mom_cutcom_plane
  global mom_pos_arc_plane

   if { [string compare "NONE" $dpp_ge(coord_rot)] || $dpp_ge(toolpath_axis_num) == "5" } {
      set mom_cycle_spindle_axis 2
      set mom_cutcom_plane  "XY"
      set mom_pos_arc_plane "XY"
   }
}


#=============================================================
proc PB_CMD_spindle_orient { } {
#=============================================================
# This command is used to add a MOM handler about positioning spindle, should be added to start of program.

if { ![llength [info commands VECTOR_ROTATE]] } {
   uplevel #0 {
      #=============================================================
      proc VECTOR_ROTATE { axis angle input_vector output_vector } {
      #=============================================================
         #This proc is used to rotating a vector about arbitrary axis.
         upvar $axis r; upvar $input_vector input ; upvar $output_vector output

         #set up matrix to rotate about an arbitrary axis
         set m(0) [expr $r(0)*$r(0)*(1-cos($angle))+cos($angle)]
         set m(1) [expr $r(0)*$r(1)*(1-cos($angle))-$r(2)*sin($angle)]
         set m(2) [expr $r(0)*$r(2)*(1-cos($angle))+$r(1)*sin($angle)]
         set m(3) [expr $r(0)*$r(1)*(1-cos($angle))+$r(2)*sin($angle)]
         set m(4) [expr $r(1)*$r(1)*(1-cos($angle))+cos($angle)]
         set m(5) [expr $r(1)*$r(2)*(1-cos($angle))-$r(0)*sin($angle)]
         set m(6) [expr $r(0)*$r(2)*(1-cos($angle))-$r(1)*sin($angle)]
         set m(7) [expr $r(1)*$r(2)*(1-cos($angle))+$r(0)*sin($angle)]
         set m(8) [expr $r(2)*$r(2)*(1-cos($angle))+cos($angle)]

         MTX3_vec_multiply input m output
         } ; #proc
      } ; #uplevel
   }

if { ![llength [info commands SPINDLE_ORIENTATION_ANGLE]] } {
   uplevel #0 {
      #=======================================================================================================================
      proc SPINDLE_ORIENTATION_ANGLE { spindle_orient_ref_axis input_angle rotate_matrix {initial_offset_angle 0} } {
      #=======================================================================================================================
         # The proc is used to calculate spindle orient output angle value.
         # It should be called in MOM_spindle_orient handler.
         #
         # Input:
         #   spindle_orient_ref_axis - X axis of feature csys respect to MCS
         #   initial_offset_angle      - initial offset angle between tool insert vector and machine X axis
         #                               the default offset angle is 0, post writer can use UDE to customize setting
         #   input_angle               - angle between desired insert direction and X axis of feature csys
         #   rotate_matrix             - matrix between MTCS and local MCS
         #
         # Return:
         #   angle value of spindle stop position
         #
         # Revisions:
         #-----------
         # 2015-04-09 Jintao - Initial implementation
         # 2015-09-25 szl    - save_mom_kin_machine_type is set whenever mom_kin_machine_type is faked into "dual-table"(for auto3d)
         # 2015-12-22 Jintao - Remove global declaration and account the rotation of feature reference vector for 3axis machine

         upvar $spindle_orient_ref_axis feature_ref_axis
         upvar $rotate_matrix matrix

         set v0 0.0; set v1 1.0

         VEC3_init v1 v0 v0 insert_ref_direction
         VEC3_init v0 v0 v0 insert_rotated_direction
         VEC3_init v0 v0 v0 intermediate_vector

         if { ![info exists ::save_mom_kin_machine_type] } {
            set machine_type $::mom_kin_machine_type

            } else {
                   set machine_type $::save_mom_kin_machine_type
                   }

         # get rotation angle in case the kinematic has been reloaded
         GET_ROT_ANGLE rot_angle

         # account rotary axis direction and zero offset
         if { [info exists ::mom_kin_4th_axis_type] } {
            if { ![string compare "SIGN_DETERMINES_DIRECTION" $::mom_kin_4th_axis_direction] } {
               set rot_angle(0) [expr abs($rot_angle(0))]
               }

            set rot_angle(0) [expr ($rot_angle(0) - $::mom_kin_4th_axis_zero) * $::DEG2RAD]

            if { [info exists ::save_mom_kin_4th_axis_vector] } {
               VMOV 3 ::save_mom_kin_4th_axis_vector fourth_axis_vector

               } else {
                      VMOV 3 ::mom_kin_4th_axis_vector fourth_axis_vector
                      }
            }

         if { [info exists ::mom_kin_5th_axis_type] } {
            if { ![string compare "SIGN_DETERMINES_DIRECTION" $::mom_kin_5th_axis_direction] } {
               set rot_angle(1) [expr abs($rot_angle(1))]
               }

            set rot_angle(1) [expr ($rot_angle(1) - $::mom_kin_5th_axis_zero) * $::DEG2RAD]

            if { [info exists ::save_mom_kin_5th_axis_vector] } {
               VMOV 3 ::save_mom_kin_5th_axis_vector fifth_axis_vector

               } else {
                      VMOV 3 ::mom_kin_5th_axis_vector fifth_axis_vector
                      }
            }

         switch $machine_type {
               5_axis_dual_head {
                                if { [info exists ::dpp_ge(coord_rot)] && ![string compare "AUTO_3D" $::dpp_ge(coord_rot)] } {
                                   set val $rot_angle(0)
                                   set rot_angle(0) $rot_angle(1)
                                   set rot_angle(1) $val
                                   }

                                VECTOR_ROTATE fifth_axis_vector $rot_angle(1) insert_ref_direction intermediate_vector
                                VECTOR_ROTATE fourth_axis_vector $rot_angle(0) intermediate_vector insert_rotated_direction
                                }
              5_axis_head_table -
              5_axis_dual_table {
                                VECTOR_ROTATE fourth_axis_vector $rot_angle(0) insert_ref_direction intermediate_vector
                                VECTOR_ROTATE fifth_axis_vector $rot_angle(1) intermediate_vector insert_rotated_direction
                                }
                    4_axis_head -
                   4_axis_table {
                                VECTOR_ROTATE fourth_axis_vector $rot_angle(0) insert_ref_direction insert_rotated_direction
                                }
                    3_axis_mill -
               3_axis_mill_turn {
                                VMOV 3 insert_ref_direction insert_rotated_direction
                                }
                        default { return 0.0 }
                              }

         MTX3_vec_multiply insert_rotated_direction matrix insert_rotated_direction_rot_mcs

         VEC3_unitize insert_rotated_direction_rot_mcs insert_rotated_direction_rot_mcs

         set dot [VEC3_dot insert_rotated_direction_rot_mcs feature_ref_axis]

         if { [EQ_is_ge $dot 1.0] } {
            set angle 0.0

            } elseif { [EQ_is_le $dot -1.0] } {
                     set angle 180.0

                     } else {
                            set angle [expr $::RAD2DEG * acos($dot)]
                            }

         VEC3_cross feature_ref_axis insert_rotated_direction_rot_mcs cross_vector

         set dot [VEC3_dot cross_vector ::mom_tool_axis]

         if {  $dot > 0.0 } { set angle [expr -1 * $angle] }
            set angle [expr $input_angle + $angle - $initial_offset_angle]
            set angle [LIMIT_ANGLE $angle]

            return $angle

         } ; #SPINDLE_ORIENTATION_ANGLE
      } ; #uplevel
   }

if { ![llength [info commands GET_ROT_LOCAL]] } {
   uplevel #0 {
      #=======================================
      proc GET_ROT_LOCAL { rot_matrix } {
      #======================================
         # If the operation is under local CSYS rotation, this proc gets the rotation matrix between current coordinate and its parent coordinate
         # Otherwise the rotation matrix is unit matrix.
         # Revisions:
         #-----------
         # 2015-04-09 Jintao - Initial implementation

         global mom_parent_csys_matrix
         global mom_kin_coordinate_system_type

         upvar $rot_matrix matrix

         if { [info exists mom_kin_coordinate_system_type] && ![string compare "CSYS" $mom_kin_coordinate_system_type] } {
            VMOV 9 mom_parent_csys_matrix matrix

            } else {
                   set matrix(0) 1; set matrix(1) 0; set matrix(2) 0;
                   set matrix(3) 0; set matrix(4) 1; set matrix(5) 0;
                   set matrix(6) 0; set matrix(7) 0; set matrix(8) 1;
                   }
         } ; #proc
      } ; #uplevel
   }

if { ![llength [info commands GET_ROT_ANGLE]] } {
   uplevel #0 {
      #============================================
      proc GET_ROT_ANGLE { rot_ang } {
      #============================================
         # This command is used to get rotary axis angle, if the operation is under local CSYS rotation, we need calculate angles, otherwise
         # it is mom_out_angle_pos
         # Revisions:
         #-----------
         # 2015-06-08 Jintao - Initial implementation

         global mom_mcs_goto
         global mom_tool_axis
         global mom_sys_leader
         global mom_out_angle_pos
         global mom_kin_machine_type
         global mom_prev_rot_ang_4th
         global mom_prev_rot_ang_5th
         global mom_parent_csys_matrix
         global mom_kin_4th_axis_leader
         global mom_kin_5th_axis_leader
         global mom_kin_4th_axis_direction
         global mom_kin_5th_axis_direction
         global mom_kin_4th_axis_min_limit
         global mom_kin_4th_axis_max_limit
         global mom_kin_5th_axis_min_limit
         global mom_kin_5th_axis_max_limit
         global mom_kin_coordinate_system_type

         upvar $rot_ang rot_angle

         set rot_angle(0) 0.0 ; set rot_angle(1) 0.0

         if { ![regexp {[0-9]*} $mom_kin_machine_type axis_num] || [EQ_is_lt $axis_num 4] } { return 0 }

         if { [info exists mom_kin_coordinate_system_type] && ![string compare "CSYS" $mom_kin_coordinate_system_type] } {
            MTX3_transpose mom_parent_csys_matrix matrix
            MTX3_vec_multiply mom_tool_axis matrix spindle_axis
            MTX3_vec_multiply mom_mcs_goto matrix mcs_goto

            if { "1" == [MOM_convert_point mcs_goto spindle_axis] } {
               global mom_result
               set i 0
               foreach value $mom_result {
                       set pos($i) $value
                       incr i
                       }
               if { ![info exists mom_prev_rot_ang_4th] } { set mom_prev_rot_ang_4th 0.0 }

               set rot_angle(0)  [ROTSET $pos(3) $mom_prev_rot_ang_4th $mom_kin_4th_axis_direction\
                                         $mom_kin_4th_axis_leader mom_sys_leader(fourth_axis)\
                                         $mom_kin_4th_axis_min_limit $mom_kin_4th_axis_max_limit]
               if { [EQ_is_ge $axis_num 5] } {
                  if { ![info exists mom_prev_rot_ang_5th] } { set mom_prev_rot_ang_5th 0.0 }

                  set rot_angle(1)  [ROTSET $pos(4) $mom_prev_rot_ang_5th $mom_kin_5th_axis_direction\
                                            $mom_kin_5th_axis_leader mom_sys_leader(fifth_axis)\
                                            $mom_kin_5th_axis_min_limit $mom_kin_5th_axis_max_limit]
                  }

               } else {
                      return 0
                      }

            } else {
                   set rot_angle(0) $mom_out_angle_pos(0)
                   set rot_angle(1) $mom_out_angle_pos(1)
                   }

         return 1

         } ; #proc
      } ; #uplevel
   }

if { ![llength [info commands MOM_spindle_orient]] } {
   uplevel #0 {
      #===============================
      proc MOM_spindle_orient { } {
      #===============================
         # In the back sinking operation, initial tool insert tip is positioned to Machine coordinate X axis positive direction,
         # which means the default value of initial offset angle is 0. the post writer can use UDE to customize offset angle
         # Revisions:
         #-----------
         # 2015-04-09 Jintao - Initial implementation

         global mom_msys_matrix
         global mom_spindle_orient_angle
         global mom_spindle_orient_ref_axis
         global mom_spindle_orient_angle_defined

         # if mom_kin_coordinate_system_type is "CSYS", then rotate_matrix is mom_parent_csys_matrix. Otherwise it is unit matrix.

         GET_ROT_LOCAL rotate_matrix

         MTX3_vec_multiply mom_spindle_orient_ref_axis mom_msys_matrix spindle_orient_ref_axis

         set mom_spindle_orient_angle [SPINDLE_ORIENTATION_ANGLE spindle_orient_ref_axis $mom_spindle_orient_angle rotate_matrix]

         MOM_force once M_spindle
         MOM_do_template spindle_orient

         } ; #MOM_spindle_orient
      } ; #uplevel
   }
}


#=============================================================
proc PB_CMD_start_of_alignment_character { } {
#=============================================================
# Output a special sequence number character. Replace the ":" with any character that you require.
# You must use the command "PB_CMD_end_of_alignment_character" to reset the sequence number back to the original setting.
#
# 07-23-2012 yaoz - Initial version
  global mom_sys_leader saved_seq_num
  set saved_seq_num $mom_sys_leader(N)
  set mom_sys_leader(N) ":"
}


#=============================================================
proc PB_CMD_start_of_operation_force_addresses { } {
#=============================================================
MOM_force once S M_spindle X Y Z fourth_axis fifth_axis F
}


#=============================================================
proc PB_CMD_start_of_program { } {
#=============================================================
}


#=============================================================
proc PB_CMD_suppress_cycle_off { } {
#=============================================================
#<02-18-08 gsl>
# Suppress output of cycle off when a cycle is simulated.
#

   global mom_user_simulated_cycle

  # If the flag has been set in a simulated cycle handler
   if { [info exists mom_user_simulated_cycle] } {
      unset mom_user_simulated_cycle
      MOM_abort_event
   }
}


#=============================================================
proc PB_CMD_suppress_linear_block_plane_code { } {
#=============================================================
# This command is to be called in the linear move event to suppress
# G_plane address when the cutcom status has not changed.
# -- Assuming G_cutcom address is modal and G_plane exists in the block
#
#<10-11-09 gsl> - New
#<01-20-11 gsl> - Force out plane code for the 1st linear move when CUTCOM is on
#<03-16-12 gsl> - Added use of CALLED_BY

# Restrict this command to be executed only by MOM_linear_move
if { ![ CALLED_BY "MOM_linear_move" ] } {
   return
   }

global mom_cutcom_status
global mom_user_prev_cutcom_status

if { ![info exists mom_cutcom_status] } {
   set mom_cutcom_status UNDEFINED
   }

if { ![info exists mom_user_prev_cutcom_status] } {
   set mom_user_prev_cutcom_status UNDEFINED
   }

# Suppress plane code when no change of CUTCOM status
if { [string match "UNDEFINED" $mom_cutcom_status] || \
     [string match $mom_user_prev_cutcom_status $mom_cutcom_status] } {
   MOM_suppress once G_plane

   } else {
          # Force out plane code for the 1st CUTCOM activation of an operation,
          # otherwise plane code will only come out when work plane has changed
          # since last activation.
          set force_1st_plane_code  "1"

          if { $force_1st_plane_code } {
             # This var should have been set in PB_first_linear_move
             global mom_sys_first_linear_move

             if { ![info exists mom_sys_first_linear_move] || $mom_sys_first_linear_move } {
                if { [string match "LEFT"  $mom_cutcom_status] ||\
                     [string match "RIGHT" $mom_cutcom_status] ||\
                     [string match "ON"    $mom_cutcom_status] } {
                   MOM_force once G_plane

                   set mom_sys_first_linear_move 0
                   }
                }
             }
          }

if { ![string match $mom_user_prev_cutcom_status $mom_cutcom_status] } {
   set mom_user_prev_cutcom_status $mom_cutcom_status
   }
}


#=============================================================
proc PB_CMD_suppress_off_address { } {
#=============================================================

}


#=============================================================
proc PB_CMD_swap_4th_5th_kinematics { } {
#=============================================================
#This proc is used to swap 4th and 5th axis kinematics variables
set kin_list { ang_offset   direction leader  incr_switch  \
               limit_action max_limit min_incr min_limit \
               plane        rotation  zero }

set kin_array_list { center_offset point vector}

foreach kin_var $kin_list {
        global mom_kin_4th_axis_$kin_var
        global mom_kin_5th_axis_$kin_var

        global save_mom_kin_4th_axis_$kin_var
        global save_mom_kin_5th_axis_$kin_var

        if { [info exists save_mom_kin_4th_axis_$kin_var] && [info exists save_mom_kin_5th_axis_$kin_var] } {
           set mom_kin_4th_axis_$kin_var [set save_mom_kin_5th_axis_[set kin_var]]
           set mom_kin_5th_axis_$kin_var [set save_mom_kin_4th_axis_[set kin_var]]
           }
        }

foreach kin_var $kin_array_list {
        global mom_kin_4th_axis_$kin_var
        global mom_kin_5th_axis_$kin_var

        global save_mom_kin_4th_axis_$kin_var
        global save_mom_kin_5th_axis_$kin_var

        if { [array exists save_mom_kin_4th_axis_$kin_var] && [array exists save_mom_kin_5th_axis_$kin_var] } {
           VMOV 3 save_mom_kin_4th_axis_$kin_var mom_kin_5th_axis_$kin_var
           VMOV 3 save_mom_kin_5th_axis_$kin_var mom_kin_4th_axis_$kin_var
           }
        }

global mom_sys_leader
global mom_sys_4th_axis_has_limits
global mom_sys_5th_axis_has_limits

global save_mom_sys_leader
global save_mom_sys_4th_axis_has_limits
global save_mom_sys_5th_axis_has_limits

if { [info exists save_mom_sys_4th_axis_has_limits] && [info exists save_mom_sys_5th_axis_has_limits] } {
   set mom_sys_4th_axis_has_limits $save_mom_sys_5th_axis_has_limits
   set mom_sys_5th_axis_has_limits $save_mom_sys_4th_axis_has_limits
   }

if { [info exists save_mom_kin_4th_axis_leader] && [info exists save_mom_kin_5th_axis_leader] } {
   set mom_sys_leader(fourth_axis) $save_mom_kin_5th_axis_leader
   set mom_sys_leader(fifth_axis) $save_mom_kin_4th_axis_leader
   }

if { [info exists save_mom_sys_leader(fourth_axis_home)] && [info exists save_mom_sys_leader(fifth_axis_home)] } {
   set mom_sys_leader(fourth_axis_home) $save_mom_sys_leader(fifth_axis_home)
   set mom_sys_leader(fifth_axis_home)  $save_mom_sys_leader(fourth_axis_home)
   }

MOM_reload_kinematics
}


#=============================================================
proc PB_CMD_tap_option_detect { } {
#=============================================================
global mom_cycle_option
global mom_spindle_speed

global pop_cycle_hole_counter

incr pop_cycle_hole_counter

if { $pop_cycle_hole_counter == 1 } {
   if { [info exists mom_cycle_option] && $mom_cycle_option == "OPTION" } {
      MOM_force once M29 S
      MOM_do_template sync_tap_invoke
      }
   }
}


#=============================================================
proc PB_CMD_tapping_g_code_string_determine { } {
#=============================================================
global mom_cycle_option
global mom_spindle_direction

global final_tap_mode

if { $mom_spindle_direction == "CLW" } {
   if { [info exists mom_cycle_option] && $mom_cycle_option == "OPTION" } {
      set final_tap_mode "84.2"

      } else {
             set final_tap_mode "84"
             }

   } elseif { $mom_spindle_direction == "CCLW" } {
            if { [info exists mom_cycle_option] && $mom_cycle_option == "OPTION" } {
               set final_tap_mode "84.3"

               } else {
                      set final_tap_mode "74"
                      }
            }
}


#=============================================================
proc PB_CMD_tapping_g_code_string_determine_for_float_tap { } {
#=============================================================
# Determine the tapping G code according to thread direction for float tap.
#
# 06-25-2013 levi - Initial version
# 08-07-2015 gsl  - "TRUE" was mistaken as TRUE (no quotes).

global mom_spindle_direction
global mom_cycle_thread_right_handed

global final_tap_mode

# Get the thread direction by feature first, if doesn't exist, get it from spindle rotation direction.
if { [info exists mom_cycle_thread_right_handed] } {
   if { $mom_cycle_thread_right_handed == "TRUE" } {
      set final_tap_mode "84"

      } else {
             set final_tap_mode "74"
             }

   } elseif { $mom_spindle_direction == "CLW" } {
            set final_tap_mode "84"

            } elseif { $mom_spindle_direction == "CCLW" } {
                     set final_tap_mode "74"
                     }
}


#=============================================================
proc PB_CMD_tapping_g_code_string_determine_for_rigid_tap { } {
#=============================================================
# Determine the tapping G code according to thread direction for rigid tap.
#
# 06-25-2013 levi - Initial version
# 08-07-2015 gsl  - "TRUE" was mistaken as TRUE (no quotes).

global mom_spindle_direction
global mom_cycle_thread_right_handed

global final_tap_mode

# Get the thread direction by feature first, if doesn't exist, get it from spindle rotation direction.
if { [info exists mom_cycle_thread_right_handed] } {
   if { $mom_cycle_thread_right_handed == "TRUE" } {
      set final_tap_mode "84.2"

      } else {
             set final_tap_mode "84.3"
             }

   } elseif { $mom_spindle_direction == "CLW" } {
            set final_tap_mode "84.2"

            } elseif { $mom_spindle_direction == "CCLW" } {
                     set final_tap_mode "84.3"
                     }
}


#=============================================================
proc PB_CMD_tool_change_force_addresses { } {
#=============================================================
MOM_force once G_adjust H X Y Z S fourth_axis fifth_axis
}


#=============================================================
proc PB_CMD_tool_data { } {
#=============================================================
global mom_tool_number mom_tool_name

MOM_set_seq_off
MOM_output_literal " "
MOM_output_literal "( *** TOOL CHANGE: T[format "%02d" $mom_tool_number]: $mom_tool_name *** )"
MOM_set_seq_on
}


#=============================================================
proc PB_CMD_tool_list { } {
#=============================================================
global mom_isv_tool_number
global mom_isv_tool_name
global mom_isv_tool_number_local
global mom_isv_tool_name_local mom_isv_tool_description
global mom_isv_tool_diameter mom_isv_tool_adjust_register

#MOM_output_literal "( )"
MOM_output_literal "(----------------- TOOL TABLE SUMMARY --------------------)"
MOM_output_literal "(TOOL-NO.   TOOL-NAME                   DIAMETER   OFFSET )"
for {set i 0} {$i < 9999 } { incr i 1 } {
     if {[info exists mom_isv_tool_number($i) ]} {
            set mom_isv_tool_number_local($mom_isv_tool_number($i)) $mom_isv_tool_number($i)
            set mom_isv_tool_name_local($mom_isv_tool_number($i)) $mom_isv_tool_name($i)
       ##     set mom_isv_tool_number($i) [format "%02d" $mom_isv_tool_number($i)]
            MOM_output_literal "(   [format "%-2d" $mom_isv_tool_number($i)]      [format "%-28s" $mom_isv_tool_name($i)] [format "%-.3f" $mom_isv_tool_diameter($i)]      [format "%-2d" $mom_isv_tool_adjust_register($i)]    )"
         }
    }

#MOM_output_literal ""
#MOM_output_literal "IN ORDER BY NUMBER:"
for {set i 0} {$i < 9999 } { incr i 1 } {
     if {[info exists mom_isv_tool_number_local($i) ]} {
            set mom_isv_tool_number_local($i) [format "%02d" $mom_isv_tool_number_local($i)]
            regsub -all {_} $mom_isv_tool_name_local($i) " " mom_isv_tool_name_local($i)
            #MOM_output_literal "( $mom_isv_tool_number_local($i)  -   $mom_isv_tool_name_local($i) )"
         }
    }
MOM_output_literal "(--------------END OF TOOL TABLE SUMMARY -----------------)"
MOM_output_literal " "
}


#=============================================================
proc PB_CMD_tool_preselect { } {
#=============================================================
  global mom_next_tool_status

   if { [info exists mom_next_tool_status] } {
      if { $mom_next_tool_status == "NEXT" } {
         MOM_do_template tool_change_2
      }
   }
}


#=============================================================
proc PB_CMD_turn_on_read_ahead { } {
#=============================================================
# Turn on read ahead.

# => Uncomment next statement to disable read ahead <=
# return

   if { ![info exists ::mom_kin_read_ahead_next_motion] ||\
        ![string match "TRUE" $::mom_kin_read_ahead_next_motion] } {

      set ::mom_kin_read_ahead_next_motion "TRUE"
      MOM_reload_kinematics
   }
}


#=============================================================
proc PB_CMD_unclamp_fifth_axis { } {
#=============================================================
#  This procedure is used by auto clamping to output the code
#  needed to unclamp the fifth axis.
#
#  Do NOT add this block to events or markers.  It is a static
#  block and it is not intended to be added to events.  Do NOT
#  change the name of this custom command.

MOM_do_template fifth_axis_unclamp
}


#=============================================================
proc PB_CMD_unclamp_fourth_axis { } {
#=============================================================
#  This procedure is used by auto clamping to output the code
#  needed to unclamp the fourth axis.
#
#  Do NOT add this block to events or markers.  It is a static
#  block and it is not intended to be added to events.  Do NOT
#  change the name of this custom command.

MOM_do_template fourth_axis_unclamp
}


#=============================================================
proc PB_CMD_unset_parameter { } {
#=============================================================
global mom_cycle_option
global mom_spindle_speed
global mom_coolant_status
global mom_sys_coolant_code

global dpp_ge
global first_hole_flag
global save_mom_spindle_speed
global mom_ude_5axis_tool_path
global first_rapid_move_status

set save_mom_spindle_speed $mom_spindle_speed

catch { unset first_hole_flag }
catch { unset mom_cycle_option }
catch { unset mom_ude_5axis_tool_path }
catch { unset first_rapid_move_status }

set dpp_ge(prev_toolpath_axis_num) $dpp_ge(toolpath_axis_num)

set mom_coolant_status        "UNDEFINED"
set mom_sys_coolant_code(OFF) "9"
}


#=============================================================
proc PB_CMD_uplevel_HAAS_fourth_axis_limits { } {
#=============================================================
global mom_kin_4th_axis_max_limit
global mom_kin_4th_axis_min_limit

global mom_haas_fourth_axis_max_limit
global mom_haas_fourth_axis_min_limit
global mom_haas_fourth_axis_limits_command_status

# Restore 4th axis limits
catch { unset mom_haas_fourth_axis_max_limit }
catch { unset mom_haas_fourth_axis_min_limit }
catch { unset mom_haas_fourth_axis_limits_command_status }

set mom_kin_4th_axis_max_limit "110.0"
set mom_kin_4th_axis_min_limit "-35.0"

MOM_reload_kinematics

uplevel #0 {
   #=============================================================
   proc MOM_haas_4th_limits { } {
   #=============================================================
      global mom_kin_4th_axis_max_limit
      global mom_kin_4th_axis_min_limit

      global mom_haas_fourth_axis_max_limit
      global mom_haas_fourth_axis_min_limit
      global mom_haas_fourth_axis_limits_command_status

      if { $mom_haas_fourth_axis_max_limit > 110.0 } {
         set mom_haas_fourth_axis_max_limit 110.0
         }

      if { $mom_haas_fourth_axis_min_limit < -35.0 } {
         set mom_haas_fourth_axis_min_limit -35.0
         }

      if { $mom_haas_fourth_axis_max_limit < 0.0 || $mom_haas_fourth_axis_min_limit > 0.0 } {
         PAUSE \
         "Wrong Fourth(B) Axis Limits. Please Redefine the limits."
         MOM_abort \
         "Wrong Fourth(B) Axis Limits."
         }

      if { [string match "ACTIVE" $mom_haas_fourth_axis_limits_command_status] } {
         set mom_kin_4th_axis_max_limit [expr abs($mom_haas_fourth_axis_max_limit)]
         set mom_kin_4th_axis_min_limit [expr abs($mom_haas_fourth_axis_min_limit) * -1]

         MOM_reload_kinematics
         }
      } ; #proc
   } ; #uplevel
}


#=============================================================
proc PB_CMD_uplevel_ROTARY_AXIS_RETRACT { } {
#=============================================================
if { ![CMD_EXIST PB_ROTARY_AXIS_RETRACT] && [CMD_EXIST ROTARY_AXIS_RETRACT] } {
   rename ROTARY_AXIS_RETRACT PB_ROTARY_AXIS_RETRACT

   } else {
          return
          }

uplevel #0 {
   #==============
   proc ROTARY_AXIS_RETRACT {} {
   #==============
      global mom_prev_pos
      global mom_prev_alt_pos
      global mom_prev_mcs_goto

      global dpp_ge

      if { $dpp_ge(toolpath_axis_num) == "5" && $dpp_ge(sys_output_coord_mode) == "TCP_FIX_TABLE" } {
         VMOV 3 mom_prev_mcs_goto mom_prev_pos
         VMOV 3 mom_prev_mcs_goto mom_prev_alt_pos
         }

      PB_ROTARY_AXIS_RETRACT
      } ; #proc
   } ; #uplevel 0
}


#-------------------------------------------------------------
proc -cosD { a } {
#-------------------------------------------------------------
return [expr -1 * cos( $::DEG2RAD * $a )]
}


#-------------------------------------------------------------
proc -sinD { a } {
#-------------------------------------------------------------
return [expr -1 * sin( $::DEG2RAD * $a )]
}


#=============================================================
proc ABORT_EVENT_CHECK { } {
#=============================================================
# Called by every motion event to abort its handler based on
# the setting of mom_sys_abort_next_event.
#
   if { [info exists ::mom_sys_abort_next_event] && $::mom_sys_abort_next_event } {
      if { [CMD_EXIST PB_CMD_kin_abort_event] } {
         PB_CMD_kin_abort_event
      }
   }
}


#=============================================================
proc ACCOUNT_HEAD_OFFSETS { POS flag } {
#=============================================================
# Command to account for the offsets of angled-head attachment.
# There'll be no effect, if head attachment is not in use or
# offsets are zeros.
#
# - Called by LOCK_AXIS & UNLOCK_AXIS
#
# Inputs:
#
#   POS  : Array name (reference) of a position
#   flag : Type of operation
#           1 = Add offsets
#           0 = Remove offsets
#
# Outputs:
#   Updated POS
#
#<04-16-2014 gsl> Inception
#

   upvar $POS pos

   global mom_kin_machine_type
   global mom_head_gauge_point

   if { [info exists mom_head_gauge_point] } {
      set len [VEC3_mag mom_head_gauge_point]

      if [EQ_is_gt $len 0.0] {
         switch $flag {
            1 {
              # Adding offsets
               VEC3_add pos mom_head_gauge_point pos
            }

            0 -
            default {
              # Subtract offsets
               VEC3_sub pos mom_head_gauge_point pos
            }
         }
      }
   }
}


#=============================================================
proc ANGLE_CHECK { a axis } {
#=============================================================
# called by ROTARY_AXIS_RETRACT
#
#   Return:
#     1: Within limits
#    -1: Out of limits
#     0: Special condition (0 ~ 360 & MAGNITUDE_DETERMINES_DIRECTION)
#

   upvar $a ang

   global mom_kin_4th_axis_max_limit
   global mom_kin_5th_axis_max_limit
   global mom_kin_4th_axis_min_limit
   global mom_kin_5th_axis_min_limit
   global mom_kin_4th_axis_direction
   global mom_kin_5th_axis_direction

   if { $axis == 4 } {
      set min $mom_kin_4th_axis_min_limit
      set max $mom_kin_4th_axis_max_limit
      set dir $mom_kin_4th_axis_direction
   } else {
      set min $mom_kin_5th_axis_min_limit
      set max $mom_kin_5th_axis_max_limit
      set dir $mom_kin_5th_axis_direction
   }

   if { [EQ_is_equal $min 0.0] && [EQ_is_equal $max 360.0] &&\
       ![string compare "MAGNITUDE_DETERMINES_DIRECTION" $dir] } {

      return 0

   } else {

      while { $ang > $max && $ang > [expr $min + 360.0] } { set ang [expr $ang - 360.0] }
      while { $ang < $min && $ang < [expr $max - 360.0] } { set ang [expr $ang + 360.0] }

      if { $ang > $max || $ang < $min } {

         return -1

      } else {

         return 1
      }
   }
}


#=============================================================
proc ARCTAN { y x } {
#=============================================================
   global PI

   if { [EQ_is_zero $y] } { set y 0 }
   if { [EQ_is_zero $x] } { set x 0 }

   if { [expr $y == 0] && [expr $x == 0] } {
      return 0
   }

   set ang [expr atan2($y,$x)]

   if { $ang < 0 } {
      return [expr $ang + $PI*2]
   } else {
      return $ang
   }
}


#=============================================================
proc ARR_sort_array_to_list { ARR {by_value 0} {by_decr 0} } {
#=============================================================
# This command will sort and build a list of elements of an array.
#
#   ARR      : Array Name
#   by_value : 0 Sort elements by names (default)
#              1 Sort elements by values
#   by_decr  : 0 Sort into increasing order (default)
#              1 Sort into decreasing order
#
#   Return a list of {name value} couplets
#
#-------------------------------------------------------------
# Feb-24-2016 gsl - Added by_decr flag
#
   upvar $ARR arr

   set list [list]
   foreach { e v } [array get arr] {
      lappend list "$e $v"
   }

   set val [lindex [lindex $list 0] $by_value]

   if { $by_decr } {
      set trend "decreasing"
   } else {
      set trend "increasing"
   }

   if [expr $::tcl_version > 8.1] {
      if [string is integer "$val"] {
         set list [lsort -integer    -$trend -index $by_value $list]
      } elseif [string is double "$val"] {
         set list [lsort -real       -$trend -index $by_value $list]
      } else {
         set list [lsort -dictionary -$trend -index $by_value $list]
      }
   } else {
      set list    [lsort -dictionary -$trend -index $by_value $list]
   }

return $list
}


#-------------------------------------------------------------
proc ARR_sort_array_vals { ARR } {
#-------------------------------------------------------------
# This command will sort and return a list containing indexed elements of an array.
#
   upvar $ARR arr

   set list [list]
   foreach a [lsort -dictionary [array names arr]] {
      if ![catch {expr $arr($a)}] {
         set val [format "%+.5f" $arr($a)]
      } else {
         set val $arr($a)
      }
      lappend list ($a) $val
   }

return $list
}


#=============================================================
proc AUTO_CLAMP { } {
#=============================================================
#  This command is used to automatically output clamp and unclamp
#  codes.  This command must be called in the the command
#  << PB_CMD_kin_before_motion >>.  By default this command will
#  output M10 or M11 to do clamping or unclamping for the 4th axis or
#  M12 or M13 for the 5th axis.
#

  # Must be called by PB_CMD_kin_before_motion
   if { ![CALLED_BY "PB_CMD_kin_before_motion"] } {
return
   }


   global mom_pos
   global mom_prev_pos

   global mom_sys_auto_clamp

   if { ![info exists mom_sys_auto_clamp] || ![string match "ON" $mom_sys_auto_clamp] } {
return
   }

   set rotary_out [EQ_is_equal $mom_pos(3) $mom_prev_pos(3)]

   AUTO_CLAMP_1 $rotary_out

   set rotary_out [EQ_is_equal $mom_pos(4) $mom_prev_pos(4)]

   AUTO_CLAMP_2 $rotary_out
}


#=============================================================
proc AUTO_CLAMP_1 { out } {
#=============================================================
# called by AUTO_CLAMP & MOM_rotate

   global clamp_rotary_fourth_status

   if { ![info exists clamp_rotary_fourth_status] ||\
       ( $out == 0 && ![string match "UNCLAMPED" $clamp_rotary_fourth_status] ) } {

      PB_CMD_unclamp_fourth_axis
      set clamp_rotary_fourth_status "UNCLAMPED"

   } elseif { $out == 1 && ![string match "CLAMPED" $clamp_rotary_fourth_status] } {

      PB_CMD_clamp_fourth_axis
      set clamp_rotary_fourth_status "CLAMPED"
   }
}


#=============================================================
proc AUTO_CLAMP_2 { out } {
#=============================================================
# called by AUTO_CLAMP & MOM_rotate

   global mom_kin_machine_type

   set machine_type [string tolower $mom_kin_machine_type]
   switch $machine_type {
      5_axis_dual_table -
      5_axis_dual_head  -
      5_axis_head_table { }

      default           {
return
      }
   }

   global clamp_rotary_fifth_status

   if { ![info exists clamp_rotary_fifth_status] ||\
        ( $out == 0 && ![string match "UNCLAMPED" $clamp_rotary_fifth_status] ) } {

      PB_CMD_unclamp_fifth_axis
      set clamp_rotary_fifth_status "UNCLAMPED"

   } elseif { $out == 1 && ![string match "CLAMPED" $clamp_rotary_fifth_status] } {

      PB_CMD_clamp_fifth_axis
      set clamp_rotary_fifth_status "CLAMPED"
   }
}


#=============================================================
proc AXIS_SET { axis } {
#=============================================================
# Called by MOM_rotate & SET_LOCK to detect if the given axis is the 4th or 5th rotary axis.
# It returns 0, if no match has been found.
#

  global mom_sys_leader

   if { ![string compare "[string index $mom_sys_leader(fourth_axis) 0]AXIS" $axis] } {
      return 3
   } elseif { ![string compare "[string index $mom_sys_leader(fifth_axis) 0]AXIS" $axis] } {
      return 4
   } else {
      return 0
   }
}


#=============================================================
proc CALCULATE_ANGLE { mode matrix ang } {
#=============================================================
# This command is used to calculate the coordinate system rotation angles
# to support coordinate rotation function (G68/ROT/AROT G68.2/CYCLE800/PLANE SPATIAL)
#
#<Feb-03-2016 gsl> This command does not seem to be used any more?

    upvar $matrix rotation_matrix
  #  upvar $a A; upvar $b B; upvar $c C
    upvar $ang rot_ang

    global RAD2DEG
    global coord_ang_A coord_ang_B coord_ang_C

    set m0 $rotation_matrix(0)
    set m1 $rotation_matrix(1)
    set m2 $rotation_matrix(2)
    set m3 $rotation_matrix(3)
    set m4 $rotation_matrix(4)
    set m5 $rotation_matrix(5)
    set m6 $rotation_matrix(6)
    set m7 $rotation_matrix(7)
    set m8 $rotation_matrix(8)

    if {$mode == "XYZ"} {
       set cos_b_sq [expr $m0*$m0 + $m3*$m3]

       if { [EQ_is_equal $cos_b_sq 0.0] } {

         set cos_b 0.0
         #set cos_c 1.0
         #set cos_a $m4
         #set sin_c 0.0
         #set sin_a [expr -1*$m5]
         set cos_a 1.0
         set sin_a 0.0
         set cos_c $m4
         set sin_c [expr -1*$m1]

         if { $m6 < 0.0 } {
           set sin_b 1.0
         } else {
           set sin_b -1.0
         }

       } else {

         set cos_b [expr sqrt($cos_b_sq)]
         set sin_b [expr -$m6]

         set cos_a [expr $m8/$cos_b]
         set sin_a [expr $m7/$cos_b]

         set cos_c [expr $m0/$cos_b]
         set sin_c [expr $m3/$cos_b]

       }

       set A [expr -atan2($sin_a,$cos_a)*$RAD2DEG]
       set B [expr -atan2($sin_b,$cos_b)*$RAD2DEG]
       set C [expr -atan2($sin_c,$cos_c)*$RAD2DEG]

       set rot_ang(0) $A; set rot_ang(1) $B; set rot_ang(2) $C

    } elseif {$mode=="ZXY"} {
       set cos_a_sq [expr $m3*$m3 + $m4*$m4]

       if { [EQ_is_equal $cos_a_sq 0.0] } {

          set cos_a 0.0
          set cos_c 1.0
          set sin_c 0.0
          set sin_b $m6
          set cos_b $m0

          if { $m5 < 0.0 } {
             set sin_a -1.0
          } else {
             set sin_a 1.0
          }

        } else {

          set cos_a [expr sqrt($cos_a_sq)]
          set sin_a [expr $m5]

          set cos_b [expr $m8/$cos_a]
          set sin_b [expr -$m2/$cos_a]

          set cos_c [expr $m4/$cos_a]
          set sin_c [expr -$m3/$cos_a]
       }

       set A [expr atan2($sin_a,$cos_a)*$RAD2DEG]
       set B [expr atan2($sin_b,$cos_b)*$RAD2DEG]
       set C [expr atan2($sin_c,$cos_c)*$RAD2DEG]
       set rot_ang(0) $C; set rot_ang(1) $A; set rot_ang(2) $B

    } elseif {$mode=="ZYX"} {
        if {[EQ_is_equal [expr abs($m2)] 1.0]} {
           set C [expr atan2([expr -1*$m3],$m4)]
        } else {
           set C [expr atan2($m1,$m0)]
        }

        set length [expr sqrt($m0*$m0 + $m1*$m1)]
        set B [expr -1*atan2($m2,$length)]
        set cos_B [expr cos($B)]

        if {![EQ_is_zero $cos_B]} {
           set A [expr atan2($m5/$cos_B,$m8/$cos_B)]
        } else {
           set A 0.0
        }

        set A [expr $A*$RAD2DEG]
        set B [expr $B*$RAD2DEG]
        set C [expr $C*$RAD2DEG]
        set rot_ang(0) $C; set rot_ang(1) $B; set rot_ang(2) $A
     } elseif {$mode=="ZXZ"} {
        set sin_b_sq [expr $m2*$m2 + $m5*$m5]

        if { [EQ_is_equal $sin_b_sq 0.0] } {
             set cos_b 1.0
             set sin_b 0.0
             set sin_c 0.0
             set cos_c 1.0
             set sin_a $m1
            # set cos_a $m0
           if {$m8>0} {
           set cos_b 1.0
           set cos_a $m0
           } else {
           set cos_b -1.0
           set cos_a -$m4
            }
        } else {
         set sin_b [expr sqrt($sin_b_sq)]
         set cos_b [expr $m8]

         set cos_a [expr -$m7/$sin_b]
         set sin_a [expr $m6/$sin_b]

         set cos_c [expr $m5/$sin_b]
         set sin_c [expr $m2/$sin_b]
      }

      set A [expr atan2($sin_a,$cos_a)*$RAD2DEG]
      set B [expr atan2($sin_b,$cos_b)*$RAD2DEG]
      set C [expr atan2($sin_c,$cos_c)*$RAD2DEG]

       set rot_ang(0) $A; set rot_ang(1) $B; set rot_ang(2) $C

    } else {
       MOM_output_to_listing_device " The mode $mode is not available!"
       set A 0
       set B 0
       set C 0
       set rot_ang(0) $A; set rot_ang(1) $B; set rot_ang(2) $C

    }

    set coord_ang_A $A
    set coord_ang_B $B
    set coord_ang_C $C
}


#=============================================================
proc CALCULATE_G68 { coord_rot matrix offset vec angle } {
#=============================================================
# General G68 solution should output G68 command twice:
# G68 X Y Z I J K R1; G68 0 0 0 0 0 1 R2
# The vector(I,J,K) is obtained by cross multiplying the Z vectors of G54 and final local csys,
# R1 is the angle between these two Z vectors. R2 is the angle between the X vector of G54 after
# rotating around (I,J,K) for R1 and the X vector of final local csys.
# Input:
#   coord_rot - coordinate rotation mode: "LOCAL", "AUTO_3D"
#   matrix
# Output:
#   offset vec angle
  upvar $matrix final_matrix
  upvar $offset linear_offset
  upvar $vec first_vec
  upvar $angle rotation_angle
  global RAD2DEG
  global dpp_ge
  global save_mom_kin_machine_type
  global save_mom_kin_4th_axis_vector
  global save_mom_kin_5th_axis_vector

# If machine has a table, should compensate the rotation and linear offset for local csys when table rotate.
# Need to recalculate the mapping matrix between G54 and local csys by left multiplying left_matrix
  if {[string match "*table*" $save_mom_kin_machine_type]} {
     global mom_init_pos
     global DEG2RAD
     global mom_csys_origin
     global dpp_ge
     global mom_out_angle_pos
     global save_mom_kin_4th_axis_point
     global save_mom_kin_5th_axis_point

# Calculate the linear offset between local csys and the rotary axis
     if {$coord_rot=="LOCAL"} {
        for { set i 0 } { $i < 3 } { incr i } {
           set dpp_4th_axis_offset($i) [expr $save_mom_kin_4th_axis_point($i)+$linear_offset($i)]
           set dpp_5th_axis_offset($i) [expr $save_mom_kin_5th_axis_point($i)+$linear_offset($i)]
         }
     } elseif {$coord_rot=="AUTO_3D"} {
        VMOV 3 save_mom_kin_4th_axis_point dpp_4th_axis_offset
        VMOV 3 save_mom_kin_5th_axis_point dpp_5th_axis_offset
     }

# Calculate cosine and sine for rotary axis in local csys condition and auto 3d condition
     if {$coord_rot=="LOCAL"} {
        set cos_4th [expr cos($DEG2RAD*$mom_init_pos(3))]
        set sin_4th [expr -sin($DEG2RAD*$mom_init_pos(3))]
        set cos_5th [expr cos($DEG2RAD*$mom_init_pos(4))]
        set sin_5th [expr -sin($DEG2RAD*$mom_init_pos(4))]
     } elseif {$coord_rot=="AUTO_3D"} {
        set cos_4th [expr cos($DEG2RAD*$mom_out_angle_pos(0))]
        set sin_4th [expr -sin($DEG2RAD*$mom_out_angle_pos(0))]
        set cos_5th [expr cos($DEG2RAD*$mom_out_angle_pos(1))]
        set sin_5th [expr -sin($DEG2RAD*$mom_out_angle_pos(1))]
     }

#  Calculate the left matrix and recalculate the mapping matrix between G54 and local csys
     CALCULATE_LEFT_MATRIX save_mom_kin_4th_axis_vector save_mom_kin_5th_axis_vector $save_mom_kin_machine_type dpp_4th_axis_offset dpp_5th_axis_offset $cos_4th $sin_4th $cos_5th $sin_5th left_matrix
     MTX4_multiply left_matrix final_matrix temp
     VMOV 16 temp final_matrix
 }

# Get 3X3 rotation matrix between G54 and local csys
    for { set i 0 } { $i < 3 } { incr i } {
        set final_csys_matrix_rot($i) $final_matrix($i)
    }
    for { set i 4 } { $i < 7 } { incr i } {
        set final_csys_matrix_rot([expr $i-1]) $final_matrix($i)
    }
    for { set i 8 } { $i < 11 } { incr i } {
        set final_csys_matrix_rot([expr $i-2]) $final_matrix($i)
    }

# Calculate the linear offset G68 should output
    set linear_offset(0) $final_matrix(12)
    set linear_offset(1) $final_matrix(13)
    set linear_offset(2) $final_matrix(14)

# Calculate the first vector G68 should output
    set u(0) 0; set u(1) 0; set u(2) 1
    MTX3_z final_csys_matrix_rot v
    if {![EQ_is_equal [VEC3_mag v] 0]} {
       VEC3_unitize v temp
       VMOV 3 temp v
    }

    VEC3_cross u v w
    if {![EQ_is_equal [VEC3_mag w] 0]} {
       VEC3_unitize w temp
       VMOV 3 temp first_vec
    } else {
       VMOV 3 w first_vec
    }

# Calculate angle for the first rotarion around first_vec vector
    set cos_rot1 [VEC3_dot u v]
    set rot1 [expr acos($cos_rot1)]

# Calculate X axis of G54 after rotating around first_vec vector
    set x_before_first_rot(0) 1;  set x_before_first_rot(1) 0;  set x_before_first_rot(2) 0
    VECTOR_ROTATE first_vec $rot1 x_before_first_rot x_after_first_rot
    if {![EQ_is_equal [VEC3_mag x_after_first_rot] 0]} {
       VEC3_unitize x_after_first_rot temp
       VMOV 3 temp x_after_first_rot
    }

# Calculate angle for the second rotarion around z axis
    MTX3_x final_csys_matrix_rot final_x
    if {![EQ_is_equal [VEC3_mag final_x] 0]} {
       VEC3_unitize final_x temp
       VMOV 3 temp final_x
    }

    set cos_rot2 [VEC3_dot x_after_first_rot final_x]
    if {$cos_rot2 > 1} {
       set cos_rot2 1
    } elseif {$cos_rot2 < -1} {
       set cos_rot2 -1
    }

    set rot2 [ expr acos($cos_rot2)]

# Detect whether rot2 need to be adjust to -rot2
    VEC3_cross x_after_first_rot final_x z_vec
    if {![EQ_is_equal [VEC3_mag z_vec] 0]} {
       VEC3_unitize z_vec temp
       VMOV 3 temp z_vec
    }

    MTX3_z final_csys_matrix_rot final_z
    if {![EQ_is_equal [VEC3_mag final_z] 0]} {
       VEC3_unitize final_z temp
       VMOV 3 temp final_z
    }

    if {![EQ_is_equal [VEC3_mag z_vec] 0] && ![VEC3_is_equal z_vec final_z]} {
       set rot2 [expr -$rot2]
    }

    set rotation_angle(0) [expr $rot1*$RAD2DEG]
    set rotation_angle(1) [expr $rot2*$RAD2DEG]
    set rotation_angle(2) 0
}


#=============================================================
proc CALCULATE_LEFT_MATRIX { 4th_axis_vector_val 5th_axis_vector_val machine_type 4th_axis_offset_val 5th_axis_offset_val cos_4th sin_4th cos_5th sin_5th left_matrix } {
#=============================================================
# Calculate the left matrix which will be used to compensate table rotation if use G68 to output
# Input:
#   4th_axis_offset_val, 5th_axis_offset_val
#   cos_4th, sin_4th, cos_5th, sin_5th
# Output:
#   left_matrix

   upvar $left_matrix dpp_left_matrix
   upvar $4th_axis_offset_val 4th_axis_offset
   upvar $5th_axis_offset_val 5th_axis_offset
   upvar $4th_axis_vector_val 4th_axis_vector
   upvar $5th_axis_vector_val 5th_axis_vector

   if { $machine_type == "5_axis_dual_table" } {
   # matrix rotating around fifth axis
      set a $5th_axis_vector(0); set b $5th_axis_vector(1); set c $5th_axis_vector(2)
      set l(0) [expr $a*$a*(1-$cos_5th)+$cos_5th]
      set l(1) [expr $a*$b*(1-$cos_5th)+$c*$sin_5th]
      set l(2) [expr $a*$c*(1-$cos_5th)-$b*$sin_5th]
      set l(3) [expr $a*$b*(1-$cos_5th)-$c*$sin_5th]
      set l(4) [expr $b*$b*(1-$cos_5th)+$cos_5th]
      set l(5) [expr $b*$c*(1-$cos_5th)+$a*$sin_5th]
      set l(6) [expr $a*$c*(1-$cos_5th)+$b*$sin_5th]
      set l(7) [expr $b*$c*(1-$cos_5th)-$a*$sin_5th]
      set l(8) [expr $c*$c*(1-$cos_5th)+$cos_5th]

   # consider the linear offset to calculate the matrix rotating around fifth axis
      set a $5th_axis_offset(0); set b $5th_axis_offset(1); set c $5th_axis_offset(2)
      set ll(0) $l(0); set ll(1) $l(1); set ll(2) $l(2); set ll(3) 0;
      set ll(4) $l(3); set ll(5) $l(4); set ll(6) $l(5); set ll(7) 0;
      set ll(8) $l(6); set ll(9) $l(7); set ll(10) $l(8);set ll(11) 0;
      set ll(12) [expr $a*(1-$l(0))-$b*$l(3)-$c*$l(6)]; set ll(13) [expr $b*(1-$l(4))-$a*$l(1)-$c*$l(7)]; set ll(14) [expr $c*(1-$l(8))-$a*$l(2)-$b*$l(5)]; set ll(15) 1;

   # matrix rotating around fourth axis
      set a $4th_axis_vector(0); set b $4th_axis_vector(1); set c $4th_axis_vector(2)
      set t(0) [expr $a*$a*(1-$cos_4th)+$cos_4th]
      set t(1) [expr $a*$b*(1-$cos_4th)+$c*$sin_4th]
      set t(2) [expr $a*$c*(1-$cos_4th)-$b*$sin_4th]
      set t(3) [expr $a*$b*(1-$cos_4th)-$c*$sin_4th]
      set t(4) [expr $b*$b*(1-$cos_4th)+$cos_4th]
      set t(5) [expr $b*$c*(1-$cos_4th)+$a*$sin_4th]
      set t(6) [expr $a*$c*(1-$cos_4th)+$b*$sin_4th]
      set t(7) [expr $b*$c*(1-$cos_4th)-$a*$sin_4th]
      set t(8) [expr $c*$c*(1-$cos_4th)+$cos_4th]

   # consider the linear offset to catcutate the matrix rotating around fourth axis
      set a $4th_axis_offset(0); set b $4th_axis_offset(1); set c $4th_axis_offset(2)
      set tt(0) $t(0); set tt(1) $t(1); set tt(2) $t(2); set tt(3) 0;
      set tt(4) $t(3); set tt(5) $t(4); set tt(6) $t(5); set tt(7) 0;
      set tt(8) $t(6); set tt(9) $t(7); set tt(10) $t(8);set tt(11) 0;
      set tt(12) [expr $a*(1-$t(0))-$b*$t(3)-$c*$t(6)]; set tt(13) [expr $b*(1-$t(4))-$a*$t(1)-$c*$t(7)]; set tt(14) [expr $c*(1-$t(8))-$a*$t(2)-$b*$t(5)]; set tt(15) 1;

   # calculate the left matrix through rotating around fifth axis, then rotating around fourth axis
      MTX4_multiply tt ll dpp_left_matrix
   } elseif {$machine_type == "5_axis_head_table"} {
   # matrix rotating around fifth axis
      set a $5th_axis_vector(0); set b $5th_axis_vector(1); set c $5th_axis_vector(2)
      set l(0) [expr $a*$a*(1-$cos_5th)+$cos_5th]
      set l(1) [expr $a*$b*(1-$cos_5th)+$c*$sin_5th]
      set l(2) [expr $a*$c*(1-$cos_5th)-$b*$sin_5th]
      set l(3) [expr $a*$b*(1-$cos_5th)-$c*$sin_5th]
      set l(4) [expr $b*$b*(1-$cos_5th)+$cos_5th]
      set l(5) [expr $b*$c*(1-$cos_5th)+$a*$sin_5th]
      set l(6) [expr $a*$c*(1-$cos_5th)+$b*$sin_5th]
      set l(7) [expr $b*$c*(1-$cos_5th)-$a*$sin_5th]
      set l(8) [expr $c*$c*(1-$cos_5th)+$cos_5th]

   # consider the linear offset to calculate the matrix rotating around fifth axis
      set a $5th_axis_offset(0); set b $5th_axis_offset(1); set c $5th_axis_offset(2)
      set ll(0) $l(0); set ll(1) $l(1); set ll(2) $l(2); set ll(3) 0;
      set ll(4) $l(3); set ll(5) $l(4); set ll(6) $l(5); set ll(7) 0;
      set ll(8) $l(6); set ll(9) $l(7); set ll(10) $l(8);set ll(11) 0;
      set ll(12) [expr $a*(1-$l(0))-$b*$l(3)-$c*$l(6)]; set ll(13) [expr $b*(1-$l(4))-$a*$l(1)-$c*$l(7)]; set ll(14) [expr $c*(1-$l(8))-$a*$l(2)-$b*$l(5)]; set ll(15) 1;
      VMOV 16 ll dpp_left_matrix
   } elseif {$machine_type == "4_axis_table"} {
   # matrix rotating around fourth axis
      set a $4th_axis_vector(0); set b $4th_axis_vector(1); set c $4th_axis_vector(2)
      set t(0) [expr $a*$a*(1-$cos_4th)+$cos_4th]
      set t(1) [expr $a*$b*(1-$cos_4th)+$c*$sin_4th]
      set t(2) [expr $a*$c*(1-$cos_4th)-$b*$sin_4th]
      set t(3) [expr $a*$b*(1-$cos_4th)-$c*$sin_4th]
      set t(4) [expr $b*$b*(1-$cos_4th)+$cos_4th]
      set t(5) [expr $b*$c*(1-$cos_4th)+$a*$sin_4th]
      set t(6) [expr $a*$c*(1-$cos_4th)+$b*$sin_4th]
      set t(7) [expr $b*$c*(1-$cos_4th)-$a*$sin_4th]
      set t(8) [expr $c*$c*(1-$cos_4th)+$cos_4th]

   # consider the linear offset to catcutate the matrix rotating around fourth axis
      set a $4th_axis_offset(0); set b $4th_axis_offset(1); set c $4th_axis_offset(2)
      set tt(0) $t(0); set tt(1) $t(1); set tt(2) $t(2); set tt(3) 0;
      set tt(4) $t(3); set tt(5) $t(4); set tt(6) $t(5); set tt(7) 0;
      set tt(8) $t(6); set tt(9) $t(7); set tt(10) $t(8);set tt(11) 0;
      set tt(12) [expr $a*(1-$t(0))-$b*$t(3)-$c*$t(6)]; set tt(13) [expr $b*(1-$t(4))-$a*$t(1)-$c*$t(7)]; set tt(14) [expr $c*(1-$t(8))-$a*$t(2)-$b*$t(5)]; set tt(15) 1;
      VMOV 16 tt dpp_left_matrix
   }

}


#=============================================================
proc CALC_CYLINDRICAL_RETRACT_POINT { refpt axis dist ret_pt } {
#=============================================================
# called by ROTARY_AXIS_RETRACT

  upvar $refpt rfp ; upvar $axis ax ; upvar $ret_pt rtp

#
# Return 0: parallel or lies on plane
#        1: unique intersection
#


#
# Create plane canonical form
#
   VMOV 3 ax plane
   set plane(3) $dist

   set num [expr $plane(3) - [VEC3_dot rfp plane]]
   set dir [VEC3_dot ax plane]

   if { [EQ_is_zero $dir] } {
return 0
   }

   for { set i 0 } { $i < 3 } { incr i } {
      set rtp($i) [expr $rfp($i) + $ax($i)*$num/$dir]
   }

return [RETRACT_POINT_CHECK rfp ax rtp]
}


#=============================================================
proc CALC_SPHERICAL_RETRACT_POINT { refpt axis cen_sphere rad_sphere int_pts } {
#=============================================================
# called by ROTARY_AXIS_RETRACT

  upvar $refpt rp ; upvar $axis ta ; upvar $cen_sphere cs
  upvar $int_pts ip

   set rad [expr $rad_sphere*$rad_sphere]
   VEC3_sub rp cs v1

   set coeff(2) 1.0
   set coeff(1) [expr ($v1(0)*$ta(0) + $v1(1)*$ta(1) + $v1(2)*$ta(2))*2.0]
   set coeff(0) [expr $v1(0)*$v1(0) + $v1(1)*$v1(1) + $v1(2)*$v1(2) - $rad]

   set num_sol [SOLVE_QUADRATIC coeff roots iroots status degree]
   if { $num_sol == 0 } { return 0 }

   if { [expr $roots(0)] > [expr $roots(1)] || $num_sol == 1 } {
      set d $roots(0)
   } else {
      set d $roots(1)
   }

   set ip(0) [expr $rp(0) + $d*$ta(0)]
   set ip(1) [expr $rp(1) + $d*$ta(1)]
   set ip(2) [expr $rp(2) + $d*$ta(2)]

return [RETRACT_POINT_CHECK rp ta ip]
}


#=============================================================
proc CALLED_BY { caller {out_warn 0} args } {
#=============================================================
# This command can be used in the beginning of a command
# to designate a specific caller for the command in question.
#
# - Users can set the optional flag "out_warn" to "1" to output
#   warning message when a command is being called by a
#   non-designated caller. By default, warning message is suppressed.
#
#  Syntax:
#    if { ![CALLED_BY "cmd_string"] } { return ;# or do something }
#  or
#    if { ![CALLED_BY "cmd_string" 1] } { return ;# To output warning }
#
# Revisions:
#-----------
# 05-25-2010 gsl - Initial implementation
# 03-09-2011 gsl - Enhanced description
# 06-29-2018 gsl - Only compare the 0th element in command string
#

   if { [info level] <= 2 } {
return 0
   }

   if { ![string compare "$caller" [lindex [info level -2] 0] ] } {
return 1
   } else {
      if { $out_warn } {
         CATCH_WARNING "\"[lindex [info level -1] 0]\" cannot be executed in \"[lindex [info level -2] 0]\". \
                        It must be called by \"$caller\"!"
      }
return 0
   }
}


#=============================================================
proc CATCH_WARNING { msg {output 1} } {
#=============================================================
# This command is called in a post to spice up the message to be output to the warning file.
#
   global mom_warning_info
   global mom_motion_event
   global mom_event_number
   global mom_motion_type
   global mom_operation_name


   if { $output == 1 } {

      set level [info level]
      set call_stack ""
      for { set i 1 } { $i < $level } { incr i } {
         set call_stack "$call_stack\[ [lindex [info level $i] 0] \]"
      }

      global mom_o_buffer
      if { ![info exists mom_o_buffer] } {
         set mom_o_buffer ""
      }

      if { ![info exists mom_motion_event] } {
         set mom_motion_event ""
      }

      if { [info exists mom_operation_name] && [string length $mom_operation_name] } {
         set mom_warning_info "$msg\n\  Operation $mom_operation_name - Event $mom_event_number [MOM_ask_event_type] :\
                               $mom_motion_event ($mom_motion_type)\n\    $mom_o_buffer\n\      $call_stack\n"
      } else {
         set mom_warning_info "$msg\n\  Event $mom_event_number [MOM_ask_event_type] :\
                               $mom_motion_event ($mom_motion_type)\n\    $mom_o_buffer\n\      $call_stack\n"
      }

      MOM_catch_warning
   }

   # Restore mom_warning_info for subsequent use
   set mom_warning_info $msg
}


#===================================================================================
proc CHECK_LICENSE { } {
#===================================================================================
   global customer_server_id
   global customer_server_id_length

   set log_file [ MOM_ask_syslog_name ]

   set load_log_file [open $log_file r]
   set log_file_read [read $load_log_file]
   close $load_log_file

   if {  [MOM_ask_env_var UGII_MAJOR_VERSION] > 12 || \
        ([MOM_ask_env_var UGII_MAJOR_VERSION] == 12 && [MOM_ask_env_var UGII_SUBMINOR_VERSION] > 0) } {
      set NXVersion "newer"

      } else {
             set NXVersion "older"
             }

   if { [string match "newer" $NXVersion] } {
      set string_to_search "License File Sold To"

      } else {
             set string_to_search "Licensing"
             }

   set licensing       [string first $string_to_search $log_file_read]
   set server_id_range [string range $log_file_read [expr $licensing - 15] [expr $licensing + 291]]
   set server          [string first ":" $server_id_range]
   set server_id       [string toupper [string range $server_id_range [expr $server + 2] [expr $server + $customer_server_id_length]]]

   set array_size [array size customer_server_id]

   set check_license_flag_list [list]

   for { set i 0 } { $i < $array_size } { incr i } {
       lappend check_license_flag_list [string match $customer_server_id($i) $server_id]
       }

   set check_license_flag [lsearch $check_license_flag_list 1]

   if { [EQ_is_equal $check_license_flag -1] } {
      return 1
      } else {
             return 0
             }
}


#=============================================================
proc CHECK_LOCK_ROTARY_AXIS { axis mtype } {
#=============================================================
# called by SET_LOCK

   global mom_sys_leader

   set is_valid 0
   set lock_axis_leader ""

   if { $mtype == 4 } {
      # For 4-axis machine, the locked rotary axis must be the fourth axis.
      if { [string match "FIFTH" $axis] } {
         return $is_valid
      }

      set lock_axis_leader [string index $mom_sys_leader(fourth_axis) 0]

   } elseif { $mtype == 5 } {
      # For 5-axis machine, the locked rotary axis must be the fifth axis.
      if { [string match "FOURTH" $axis] } {
         return $is_valid
      }

      set lock_axis_leader [string index $mom_sys_leader(fifth_axis) 0]

   } else {

      return $is_valid
   }

   # Handle the case when axis is "AAXIS", "BAXIS" or "CAXIS"
   set cur_axis_leader [string index $axis 0]
   switch $cur_axis_leader {
      A -
      B -
      C {
         if { [string match $lock_axis_leader $cur_axis_leader] } {
            # The specified rotary axis is valid
            set is_valid 1
         }
      }
      default {
         set is_valid 1
      }
   }

   return $is_valid
}


#=============================================================
proc CMD_EXIST { cmd {out_warn 0} args } {
#=============================================================
# This command can be used to detect the existence of a command
# before executing it.
# - Users can set the optional flag "out_warn" to "1" to output
#   warning message when a command to be called doesn't exist.
#   By default, warning message is suppressed.
#
#  Syntax:
#    if { [CMD_EXIST "cmd_string"] } { cmd_string }
#  or
#    if { [CMD_EXIST "cmd_string" 1] } { cmd_string ;# To output warning }
#
# Revisions:
#-----------
# 05-25-10 gsl - Initial implementation
# 03-09-11 gsl - Enhanced description
#

   if { [llength [info commands "$cmd"] ] } {
return 1
   } else {
      if { $out_warn } {
         CATCH_WARNING "Command \"$cmd\" called by \"[lindex [info level -1] 0]\" does not exist!"
      }
return 0
   }
}


#=============================================================================
proc COMPARE_NX_VERSION { this_ver target_ver } {
#=============================================================================
# Compare a given NX version with target version.
# ==> Number of fields will be compared based on the number of "." contained in target.
#
# Return 1: Newer
#        0: Same
#       -1: Older
#

   set vlist_1 [split $this_ver   "."]
   set vlist_2 [split $target_ver "."]

   set ver_check 0

   set idx 0
   foreach v2 $vlist_2 {

      if { $ver_check == 0 } {
         set v1 [lindex $vlist_1 $idx]
         if { $v1 > $v2 } {
            set ver_check 1
         } elseif { $v1 == $v2 } {
            set ver_check 0
         } else {
            set ver_check -1
         }
      }

      if { $ver_check != 0 } {
         break
      }

      incr idx
   }

return $ver_check
}


#-------------------------------------------------------------
proc CSYS_ask_rotary_angles { ANG_A ANG_B ANG_C CSYS } {
#-------------------------------------------------------------
# Find rotations of a orthogonal machine from given CSYS matrix.
#
# For a 5-axis machine, there will be 2 angles most (with 1 zero);
# one of them is the 4th-axis rotation which should be performed first!
#
#
# Jan 24, 2013 gsl -
# Jun 04, 2013 gsl - initialize angles
# Apr 18, 2019 gsl - 5-axis case was not complete.
#
   upvar $ANG_A ang_A
   upvar $ANG_B ang_B
   upvar $ANG_C ang_C
   upvar $CSYS  csys

   set ang_A 0.0
   set ang_B 0.0
   set ang_C 0.0

   global mom_kin_machine_type
   global mom_kin_4th_axis_plane mom_kin_5th_axis_plane

   # Do we ever need this for lathe?
   if [string match "MILL" [PB_CMD_ask_machine_type]] {

      # Principal axes: (X/Y/Z)
      set px(0) 1.0;  set px(1) 0.0;  set px(2) 0.0;
      set py(0) 0.0;  set py(1) 1.0;  set py(2) 0.0;
      set pz(0) 0.0;  set pz(1) 0.0;  set pz(2) 1.0;

      # CSYS matrix: (X'/Y'/Z')
      set ppx(0) $csys(0)
      set ppx(1) $csys(1)
      set ppx(2) $csys(2)

      set ppy(0) $csys(3)
      set ppy(1) $csys(4)
      set ppy(2) $csys(5)

      set ppz(0) $csys(6)
      set ppz(1) $csys(7)
      set ppz(2) $csys(8)


      if { [string match "4*" $mom_kin_machine_type] } {

         switch $mom_kin_4th_axis_plane {
            "XY" {
               set ang_C [expr $::RAD2DEG * atan2($ppx(1),$ppx(0))]
            }
            "YZ" {
               set ang_A [expr $::RAD2DEG * atan2(-1*$ppz(1),$ppz(2))]
            }
            "ZX" {
               set ang_B [expr $::RAD2DEG * atan2($ppz(0),$ppz(2))]
            }
         }
      }

      if { [string match "5*" $mom_kin_machine_type] } {

         switch $mom_kin_4th_axis_plane {
            "XY" {
              # Find principal angle of X' on XY plane to rotate about Y(B).
               set ang_B [VEC3_find_principal_angle ppx XY]

              # If X' is on XY plane already, find angle of Y' instead to rotate about X(A).
               if [EQ_is_zero $ang_B] {
                  set ang_A [VEC3_find_principal_angle ppy XY]
               }

              # Find angle from projected X' on XY to X to rotate about Z(C).
               set v(0) $ppx(0)
               set v(1) $ppx(1)
               set v(2) 0.0
               VEC3_unitize v u
               if [EQ_is_gt [VEC3_mag u] 0.0] {
                  set ang_C [expr { $::RAD2DEG * atan2(-$u(1),$u(0)) }]
               }
            }

            "YZ" {
              # Find principal angle of Y' on YZ plane to rotate about Z.
               set ang_C [VEC3_find_principal_angle ppy YZ]

              # If Y' is on YZ plane already, find angle of Z' on YZ plane.
               if [EQ_is_zero $ang_C] {
                  set ang_B [VEC3_find_principal_angle ppz YZ]
               }

              # Find angle from projected Y' on YZ to Y.
               set v(0) 0.0
               set v(1) $ppy(1)
               set v(2) $ppy(2)
               VEC3_unitize v u
               if [EQ_is_gt [VEC3_mag u] 0.0] {
                  set ang_A [expr { $::RAD2DEG * atan2(-$u(2),$u(1)) }]
               }
            }

            "ZX" {

              # Find principal angle of Z' on ZX plane to rotate about X.
               set ang_A [VEC3_find_principal_angle ppz ZX]
               if { [EQ_is_zero $ang_A] && [EQ_is_equal [VEC3_dot ppz pz] -1.0] } {
                  set ang_A 180.0
               }

              # If Z' is on ZX plane already, find angle of X' instead.
               if [EQ_is_zero $ang_A] {
                  set ang_C [VEC3_find_principal_angle ppx ZX]
               }
               if { [EQ_is_zero $ang_C] && [EQ_is_equal [VEC3_dot ppx px] -1.0] } {
                  set ang_C 180.0
               } else {
                  set ang_C [expr -$ang_C] ;# Not sure why C needs to be negated???
               }

              # Find angle from projected Z' on ZX to Z. to rotate about Y.
               set v(0) $ppz(0)
               set v(1) 0.0
               set v(2) $ppz(2)
               VEC3_unitize v u
               if [EQ_is_gt [VEC3_mag u] 0.0] {
                  set ang_B [expr { $::RAD2DEG * atan2(-$u(0),$u(2)) }]
               }
            }
         }

         switch $mom_kin_5th_axis_plane {
            "XY" {
               set ang_C [expr $::RAD2DEG * atan2($ppx(1),$ppx(0))]
            }
            "YZ" {
               set ang_A [expr $::RAD2DEG * atan2(-1*$ppz(1),$ppz(2))]
            }
            "ZX" {
               set ang_B [expr $::RAD2DEG * atan2($ppz(0),$ppz(2))]
            }
         }
      }
   }
}


#-------------------------------------------------------------
proc CSYS_diff { CSYS_REF CSYS_TGT CSYS_OFF } {
#-------------------------------------------------------------
# Find offset (matrix) from one CSYS to another.
#
   upvar $CSYS_REF c1 ;# Reference CSYS
   upvar $CSYS_TGT c2 ;# Target CSYS
   upvar $CSYS_OFF c3 ;# Offset matrix

    set u(0) [expr $c2(9)  - $c1(9)]
    set u(1) [expr $c2(10) - $c1(10)]
    set u(2) [expr $c2(11) - $c1(11)]

   MTX3_vec_multiply u c1 w

   MTX3_invert c1 c1i
   MTX3_multiply c1i c2 c3

   set c3(9)  $w(0)
   set c3(10) $w(1)
   set c3(11) $w(2)
}


#-------------------------------------------------------------
proc CSYS_match { CSYS1 CSYS2 CDIFF } {
#-------------------------------------------------------------
# Compare 2 CSYS matrix -
#
# It returns 1 when two matrices are identical and returns 0 otherwise.
# It also stores the differences in an array of 12.
#
   upvar $CSYS1 c1
   upvar $CSYS2 c2
   upvar $CDIFF c3

   set is_equal 1

   for { set i 0 } { $i < 12 } { incr i } {
      set c3($i) [expr $c1($i) - $c2($i)]
      if { $is_equal && ![EQ_is_zero $c3($i)] } {
         set is_equal 0  ;# Do not break, need to return entire DIFF matrix.
      }
   }

return $is_equal
}


#-----------------------------------------------------------
proc CSYS_offset { CSYS_REF CSYS_TGT CSYS_OFF } {
#-----------------------------------------------------------
# Find offset (matrix) from one CSYS to another.
#
   upvar $CSYS_REF c1 ;# Reference CSYS
   upvar $CSYS_TGT c2 ;# Target CSYS
   upvar $CSYS_OFF c3 ;# Offset matrix

    set u(0) [expr $c2(9)  - $c1(9)]
    set u(1) [expr $c2(10) - $c1(10)]
    set u(2) [expr $c2(11) - $c1(11)]

   MTX3_vec_multiply u c1 w

   MTX3_invert c1 c1i
   MTX3_multiply c1i c2 c3

   set c3(9)  $w(0)
   set c3(10) $w(1)
   set c3(11) $w(2)
}


#=============================================================
proc DELAY_TIME_SET { } {
#=============================================================
  global mom_sys_delay_param mom_delay_value
  global mom_delay_revs mom_delay_mode delay_time

   # Post Builder provided format for the current mode:
    if { [info exists mom_sys_delay_param(${mom_delay_mode},format)] != 0 } {
      MOM_set_address_format dwell $mom_sys_delay_param(${mom_delay_mode},format)
    }

    switch $mom_delay_mode {
      SECONDS { set delay_time $mom_delay_value }
      default { set delay_time $mom_delay_revs  }
    }
}


#=============================================================================
proc DO_BLOCK { block args } {
#=============================================================================
# May-10-2017 gsl - New (PB v12.0)
#
   set option [lindex $args 0]

   if { [CMD_EXIST MOM_has_definition_element] } {
      if { [MOM_has_definition_element BLOCK $block] } {
         if { $option == "" } {
            return [MOM_do_template $block]
         } else {
            return [MOM_do_template $block $option]
         }
      } else {
         CATCH_WARNING "Block template $block not found."
      }
   } else {
      if { $option == "" } {
         return [MOM_do_template $block]
      } else {
         return [MOM_do_template $block $option]
      }
   }
}


#=============================================================
proc DPP_GE_CALCULATE_COOR_ROT_ANGLE { mode MATRIX ANG } {
#=============================================================
# The command can be used to to calculate the coordinate system rotation angles
# and support coordinate rotation functions such as G68/ROT/AROT G68.2/CYCLE800/PLANE SPATIAL.
#
# Input:
#   mode   - Coordinate rotation mode: XYZ, ZXY, ZXZ, ZYX
#   MATRIX - Reference to an array of (0:8) defining a local coordinate system of 3x3 matrix
#
# Output:
#   ANG    - Reference to an array of (0:2) defining rotation angles in order
#            ang(0) - first rotation angle value
#            ang(1) - second rotation angle value
#            ang(2) - third rotation angle value
#
# Return:
#   1 - mode is available, 0 - mode is not available
#
# Revisions:
#-----------
# 2013-05-22 lili - Initial implementation
# 2106-02-02 gsl  - Clean up
# 08-09-2016 lili - Fix ZYX rotation angle calculation issue. zero resolution was too high.
#

   upvar $MATRIX rotation_matrix
   upvar $ANG rot_ang

   global RAD2DEG

   set m0 $rotation_matrix(0)
   set m1 $rotation_matrix(1)
   set m2 $rotation_matrix(2)
   set m3 $rotation_matrix(3)
   set m4 $rotation_matrix(4)
   set m5 $rotation_matrix(5)
   set m6 $rotation_matrix(6)
   set m7 $rotation_matrix(7)
   set m8 $rotation_matrix(8)

   set status UNDEFINED

   if { $mode == "XYZ" } {

      set cos_b_sq [expr $m0*$m0 + $m3*$m3]

      if { [EQ_is_equal $cos_b_sq 0.0] } {

         set cos_b 0.0
         set cos_a 1.0
         set sin_a 0.0
         set cos_c $m4
         set sin_c [expr -1*$m1]

         if { [expr $m6 < 0.0] } {
            set sin_b 1.0
         } else {
            set sin_b -1.0
         }

      } else {

         set cos_b [expr sqrt($cos_b_sq)]
         set sin_b [expr -$m6]

         set cos_a [expr $m8/$cos_b]
         set sin_a [expr $m7/$cos_b]

         set cos_c [expr $m0/$cos_b]
         set sin_c [expr $m3/$cos_b]
      }

      set A [expr -atan2($sin_a,$cos_a)*$RAD2DEG]
      set B [expr -atan2($sin_b,$cos_b)*$RAD2DEG]
      set C [expr -atan2($sin_c,$cos_c)*$RAD2DEG]

      set rot_ang(0) $A; set rot_ang(1) $B; set rot_ang(2) $C
      set status OK

   } elseif { $mode == "ZXY" } {

      set cos_a_sq [expr $m3*$m3 + $m4*$m4]

      if { [EQ_is_equal $cos_a_sq 0.0] } {

         set cos_a 0.0
         set cos_c 1.0
         set sin_c 0.0
         set sin_b $m6
         set cos_b $m0

         if { [expr $m5 < 0.0] } {
            set sin_a -1.0
         } else {
            set sin_a 1.0
         }

      } else {

         set cos_a [expr sqrt($cos_a_sq)]
         set sin_a [expr $m5]

         set cos_b [expr $m8/$cos_a]
         set sin_b [expr -$m2/$cos_a]

         set cos_c [expr $m4/$cos_a]
         set sin_c [expr -$m3/$cos_a]
      }

      set A [expr atan2($sin_a,$cos_a)*$RAD2DEG]
      set B [expr atan2($sin_b,$cos_b)*$RAD2DEG]
      set C [expr atan2($sin_c,$cos_c)*$RAD2DEG]

      set rot_ang(0) $C; set rot_ang(1) $A; set rot_ang(2) $B
      set status OK

   } elseif { $mode == "ZYX" } {

     #<Aug-15-2016 gsl> Replaced with new revision of <08-09-2016 lili>
     if 0 {

      if { [EQ_is_equal [expr abs($m2)] 1.0] } {
         set C [expr atan2([expr -1*$m3],$m4)]
      } else {
         set C [expr atan2($m1,$m0)]
      }

      set length [expr sqrt($m0*$m0 + $m1*$m1)]
      set B [expr -1*atan2($m2,$length)]
      set cos_B [expr cos($B)]

      if { ![EQ_is_zero $cos_B] } {
         set A [expr atan2($m5/$cos_B,$m8/$cos_B)]
      } else {
         set A 0.0
      }

      set A [expr $A*$RAD2DEG]
      set B [expr $B*$RAD2DEG]
      set C [expr $C*$RAD2DEG]

     } else {

      set cos_b_sq [expr $m0*$m0 + $m1*$m1]

      if { [EQ_is_equal $cos_b_sq 0.0] } {

         set cos_b 0.0
         set cos_a 1.0
         set sin_a 0.0
         set sin_c [expr -1*$m3]
         set cos_c $m4

         if { $m2 < 0.0 } {
            set sin_b 1.0
         } else {
            set sin_b -1.0
         }

      } else {

         set cos_b [expr sqrt($cos_b_sq)]
         set sin_b [expr -1*$m2]

         set cos_a [expr $m8/$cos_b]
         set sin_a [expr $m5/$cos_b]

         set cos_c [expr $m0/$cos_b]
         set sin_c [expr $m1/$cos_b]
      }

      set A [expr atan2($sin_a,$cos_a)*$RAD2DEG]
      set B [expr atan2($sin_b,$cos_b)*$RAD2DEG]
      set C [expr atan2($sin_c,$cos_c)*$RAD2DEG]

     }  ;# revision of <08-09-2016 lili>

      set rot_ang(0) $C; set rot_ang(1) $B; set rot_ang(2) $A
      set status OK

   } elseif { $mode == "ZXZ" } {

      set sin_b_sq [expr $m2*$m2 + $m5*$m5]

      if { [EQ_is_equal $sin_b_sq 0.0] } {

         set cos_b 1.0
         set sin_b 0.0
         set sin_c 0.0
         set cos_c 1.0
         set sin_a $m1

         if { [expr $m8 > 0.0] } {
            set cos_b 1.0
            set cos_a $m0
         } else {
            set cos_b -1.0
            set cos_a -$m4
         }

      } else {

         set sin_b [expr sqrt($sin_b_sq)]
         set cos_b [expr $m8]

         set cos_a [expr -$m7/$sin_b]
         set sin_a [expr $m6/$sin_b]

         set cos_c [expr $m5/$sin_b]
         set sin_c [expr $m2/$sin_b]
      }

      set A [expr atan2($sin_a,$cos_a)*$RAD2DEG]
      set B [expr atan2($sin_b,$cos_b)*$RAD2DEG]
      set C [expr atan2($sin_c,$cos_c)*$RAD2DEG]

      set rot_ang(0) $A; set rot_ang(1) $B; set rot_ang(2) $C
      set status OK
   }

   if { $status == "OK" } {
 return 1
   } else {
 return 0
   }
}


#=============================================================
proc DPP_GE_CALCULATE_COOR_ROT_ANGLE-X { mode matrix ang } {
#=============================================================
# The proc is used to is used to calculate the coordinate system rotation angles
# and support coordinate rotation function (G68/ROT/AROT G68.2/CYCLE800/PLANE SPATIAL).
#
# Input:
#   mode   - coordinate rotation mode. Possible value: XYZ, ZXY, ZXZ, ZYX
#   matrix - Local coordinate system 3x3 matrix
#
# Output:
#   ang    - rotation angle array, the order is rotation order
#            ang(0) - first rotation angle value
#            ang(1) - second rotation angle value
#            ang(2) - third rotation angle value
#
# Return:
#   1 - mode is available, 0 - mode is not available
#
# Revisions:
#-----------
# 2013-05-22 lili - Initial implementation
#

    upvar $matrix rotation_matrix
    upvar $ang rot_ang

    global RAD2DEG

    set m0 $rotation_matrix(0)
    set m1 $rotation_matrix(1)
    set m2 $rotation_matrix(2)
    set m3 $rotation_matrix(3)
    set m4 $rotation_matrix(4)
    set m5 $rotation_matrix(5)
    set m6 $rotation_matrix(6)
    set m7 $rotation_matrix(7)
    set m8 $rotation_matrix(8)

    if {$mode == "XYZ"} {
       set cos_b_sq [expr $m0*$m0 + $m3*$m3]

       if { [EQ_is_equal $cos_b_sq 0.0] } {

         set cos_b 0.0
         set cos_a 1.0
         set sin_a 0.0
         set cos_c $m4
         set sin_c [expr -1*$m1]

         if { $m6 < 0.0 } {
           set sin_b 1.0
         } else {
           set sin_b -1.0
         }

       } else {

         set cos_b [expr sqrt($cos_b_sq)]
         set sin_b [expr -$m6]

         set cos_a [expr $m8/$cos_b]
         set sin_a [expr $m7/$cos_b]

         set cos_c [expr $m0/$cos_b]
         set sin_c [expr $m3/$cos_b]

       }

       set A [expr -atan2($sin_a,$cos_a)*$RAD2DEG]
       set B [expr -atan2($sin_b,$cos_b)*$RAD2DEG]
       set C [expr -atan2($sin_c,$cos_c)*$RAD2DEG]

       set rot_ang(0) $A; set rot_ang(1) $B; set rot_ang(2) $C
  return 1
    } elseif {$mode=="ZXY"} {
       set cos_a_sq [expr $m3*$m3 + $m4*$m4]

       if { [EQ_is_equal $cos_a_sq 0.0] } {

          set cos_a 0.0
          set cos_c 1.0
          set sin_c 0.0
          set sin_b $m6
          set cos_b $m0

          if { $m5 < 0.0 } {
             set sin_a -1.0
          } else {
             set sin_a 1.0
          }

        } else {

          set cos_a [expr sqrt($cos_a_sq)]
          set sin_a [expr $m5]

          set cos_b [expr $m8/$cos_a]
          set sin_b [expr -$m2/$cos_a]

          set cos_c [expr $m4/$cos_a]
          set sin_c [expr -$m3/$cos_a]
       }

       set A [expr atan2($sin_a,$cos_a)*$RAD2DEG]
       set B [expr atan2($sin_b,$cos_b)*$RAD2DEG]
       set C [expr atan2($sin_c,$cos_c)*$RAD2DEG]
       set rot_ang(0) $C; set rot_ang(1) $A; set rot_ang(2) $B
  return 1
    } elseif {$mode=="ZYX"} {
        if {[EQ_is_equal [expr abs($m2)] 1.0]} {
           set C [expr atan2([expr -1*$m3],$m4)]
        } else {
           set C [expr atan2($m1,$m0)]
        }

        set length [expr sqrt($m0*$m0 + $m1*$m1)]
        set B [expr -1*atan2($m2,$length)]
        set cos_B [expr cos($B)]

        if {![EQ_is_zero $cos_B]} {
           set A [expr atan2($m5/$cos_B,$m8/$cos_B)]
        } else {
           set A 0.0
        }

        set A [expr $A*$RAD2DEG]
        set B [expr $B*$RAD2DEG]
        set C [expr $C*$RAD2DEG]
        set rot_ang(0) $C; set rot_ang(1) $B; set rot_ang(2) $A
     } elseif {$mode=="ZXZ"} {
        set sin_b_sq [expr $m2*$m2 + $m5*$m5]

        if { [EQ_is_equal $sin_b_sq 0.0] } {
           set cos_b 1.0
           set sin_b 0.0
           set sin_c 0.0
           set cos_c 1.0
           set sin_a $m1
           if {$m8>0} {
              set cos_b 1.0
              set cos_a $m0
           } else {
              set cos_b -1.0
              set cos_a -$m4
           }
        } else {
           set sin_b [expr sqrt($sin_b_sq)]
           set cos_b [expr $m8]

           set cos_a [expr -$m7/$sin_b]
           set sin_a [expr $m6/$sin_b]

           set cos_c [expr $m5/$sin_b]
           set sin_c [expr $m2/$sin_b]
        }

        set A [expr atan2($sin_a,$cos_a)*$RAD2DEG]
        set B [expr atan2($sin_b,$cos_b)*$RAD2DEG]
        set C [expr atan2($sin_c,$cos_c)*$RAD2DEG]

        set rot_ang(0) $A; set rot_ang(1) $B; set rot_ang(2) $C
  return 1
    } else {
  return 0
    }
}


#=============================================================
proc DPP_GE_COOR_ROT { ang_mode rot_angle offset pos } {
#=============================================================
# This procedure is used to detect if an operation has coordinate rotation.
# DPP_GE_COOR_ROT_LOCAL, DPP_GE_COOR_ROT_AUTO3D, DPP_GE_CALCULATE_COOR_ROT_ANGLE are called in this proc.
#
# Input:
#   ang_mode -  coordinate matrix rotation method. Possible value: XYZ, ZXY, ZXZ, ZYX
#
# Output:
#   rot_angle - rotation angle array
#               rot_angle(0) is first rotation angle value
#               rot_angle(1) is second rotation angle value
#               rot_angle(2) is third rotation angle value
#   coord_offset - linear coordinate offset from current local CSYS rotation coordinate to parent coordinate.
#   pos - linear axes position respect to rotated cooridnate
#
# Return:
#   detected coordinate mode. Possible value: NONE, LOCAL, AUTO_3D
#   NONE - no coordinate rotation
#   LOCAL - coordinate rotation set up by LOCAL CSYS MCS
#   AUTO_3D - coordinate rotation set up by tilt work plane
#
# Revisions:
#-----------
# 2013-05-22 lili - Initial implementation
#

   upvar $rot_angle angle
   upvar $pos rot_pos
   upvar $offset coord_offset

   global mom_pos
   VMOV 3 mom_pos rot_pos

   set v0 0
   VEC3_init v0 v0 v0 coord_offset

   if { [DPP_GE_COOR_ROT_LOCAL rot_matrix coord_offset] } {
      set coord_rot "LOCAL"
   } elseif { [DPP_GE_COOR_ROT_AUTO3D rot_matrix rot_pos] } {
      set coord_rot "AUTO_3D"
   } else {
      set coord_rot "NONE"
   }

   if { [string compare "NONE" $coord_rot] } {
      DPP_GE_CALCULATE_COOR_ROT_ANGLE $ang_mode rot_matrix angle
   } else {
      set angle(0) 0.0
      set angle(1) 0.0
      set angle(2) 0.0
   }
   return $coord_rot
}


#=============================================================
proc DPP_GE_COOR_ROT_AUTO3D { rot_matrix rot_pos } {
#=============================================================
# This procedure is used to detect if operation is 3+2 operation without Local CSYS rotation coordinate system.
# It will return rotation matrix and current position respect to rotated coordinate system. The machine kinemtaics
# will be reloaded to dual table machine.
# Rotation matrix calculated by current rotary axes angle and 4th,5th axis vector.
#
# Output:
#   rot_matrix - rotation matrix, mapping from the current rotation coordinate to parent coordinate(G54,G55..).
#   rot_pos    - current position respect to rotated coordinate system
#   * kinemtics - machine's kinematics will be reloaded to 5_axis_dual_table
#
# Return:
#   1 - operation is 3+2 operation without Local CSYS rotation coordinate system.
#   0 - operation is not 3+2 operation without Local CSYS rotation coordinate system.
#
# Revisions:
#-----------
# 2013-05-22 lili - Initial implementation
# 2013-10-16 levi - Reload mom_prev_pos when exchanging the pos of 4th axis and 5th axis for dual head machine.
# 2014-12-04 lili - Fix mom_kin_xth_axis_zero issue.
# 2018-04-02 ljt  - 9080157, take accout of reversed direction

   upvar $rot_matrix matrix
   upvar $rot_pos pos

   global mom_kin_coordinate_system_type
   global mom_kin_machine_type
   global mom_kin_4th_axis_point mom_kin_5th_axis_point
   global mom_kin_4th_axis_vector mom_kin_5th_axis_vector
   global mom_out_angle_pos mom_prev_out_angle_pos
   global DEG2RAD
   global mom_pos mom_mcs_goto mom_prev_pos
   global mom_kin_4th_axis_zero mom_kin_5th_axis_zero
   global mom_tool_z_offset mom_spindle_axis

   set v0 0.0; set v1 1.0
   VEC3_init v1 v0 v0 X
   VEC3_init v0 v1 v0 Y
   VEC3_init v0 v0 v1 Z
   MTX3_init_x_y_z X Y Z matrix
   VMOV 3 mom_pos pos

   if { ![string match "*5_axis*" $mom_kin_machine_type] } {
      return 0
   }

   if { [info exists mom_kin_coordinate_system_type] && ![string compare "CSYS" $mom_kin_coordinate_system_type] } {
      return 0
   } else {
      if { (![EQ_is_zero $mom_out_angle_pos(0)] && ![VEC3_is_equal mom_kin_4th_axis_vector Z]) || \
           (![EQ_is_zero $mom_out_angle_pos(1)] && ![VEC3_is_equal mom_kin_5th_axis_vector Z]) } {
      } else {
         return 0
      }
   }

  # Save kinematics
   DPP_GE_SAVE_KINEMATICS

  # get rotation angle
   if { [string match "5_axis_dual_head" $mom_kin_machine_type] } {

     # Swap rotary axes kinematics for dual head machine
      DPP_GE_SWAP_4TH_5TH_KINEMATICS

      set ang_pos(0) $mom_out_angle_pos(1)
      set ang_pos(1) $mom_out_angle_pos(0)

     # Swap rotary axes value due to kinemtics switched
      set mom_out_angle_pos(0) $ang_pos(0)
      set mom_out_angle_pos(1) $ang_pos(1)
      set mom_prev_out_angle_pos(0) $ang_pos(0)
      set mom_prev_out_angle_pos(1) $ang_pos(1)

     #<Nov-27-2018 gsl>
     if 0 {
      set mom_pos(3) $ang_pos(0)
      set mom_pos(4) $ang_pos(1)
      set mom_prev_pos(3) $ang_pos(0)
      set mom_prev_pos(4) $ang_pos(1)
     } else {
      set temp $mom_pos(3)
      set mom_pos(3) $mom_pos(4)
      set mom_pos(4) $temp
     #<Nov-28-2018 gsl> This change affects output for 8553652
     # set mom_prev_pos(3) $mom_pos(3)
     # set mom_prev_pos(4) $mom_pos(4)
      set temp $mom_prev_pos(3)
      set mom_prev_pos(3) $mom_prev_pos(4)
      set mom_prev_pos(4) $temp
     }
      MOM_reload_variable -a mom_out_angle_pos
      MOM_reload_variable -a mom_prev_out_angle_pos
      MOM_reload_variable -a mom_pos
      MOM_reload_variable -a mom_prev_pos
   } else {
      set ang_pos(0) $mom_out_angle_pos(0)
      set ang_pos(1) $mom_out_angle_pos(1)
   }

   set rot0 [expr ($ang_pos(0)-$mom_kin_4th_axis_zero)*$DEG2RAD]
   set rot1 [expr ($ang_pos(1)-$mom_kin_5th_axis_zero)*$DEG2RAD]

  # Reolad kinematics to dual-table machine
   if { ![string match "5_axis_dual_table" $mom_kin_machine_type] } {
      set mom_kin_machine_type "5_axis_dual_table"
   }

   set v0 0.0
   VEC3_init v0 v0 v0 mom_kin_4th_axis_point
   VEC3_init v0 v0 v0 mom_kin_5th_axis_point
   MOM_reload_kinematics

  # Get current position respect to rotated coordinate
   set rot_dir_4th -1
   if { [string match "reverse" $::mom_kin_4th_axis_rotation] } {
      set rot_dir_4th 1
   }
   set rot_dir_5th -1
   if { [string match "reverse" $::mom_kin_5th_axis_rotation] } {
      set rot_dir_5th 1
   }


   VECTOR_ROTATE mom_kin_5th_axis_vector [expr $rot_dir_5th*$rot1] mom_mcs_goto V
   VECTOR_ROTATE mom_kin_4th_axis_vector [expr $rot_dir_4th*$rot0] V pos

   if { [info exists mom_tool_z_offset] && [info exists mom_spindle_axis] } {
       set tool_tip_scale [expr -1*$mom_tool_z_offset]
       VEC3_scale tool_tip_scale mom_spindle_axis tool_tip
       VEC3_sub pos tool_tip pos
   }

  if 0 {
   # Recalculate Z value for driling cycle initial move without clearance plane.
   global cycle_init_flag mom_current_motion mom_cycle_rapid_to
   if {[info exists cycle_init_flag] && $cycle_init_flag == "TRUE"} {
      if { ![string compare "initial_move" $mom_current_motion]} {
         set pos(2) [expr $pos(2) + $mom_cycle_rapid_to]
      }
   }
  }

  # Calculate rotation matrix
   VECTOR_ROTATE mom_kin_4th_axis_vector $rot0 X V1
   VECTOR_ROTATE mom_kin_4th_axis_vector $rot0 Y V2
   VECTOR_ROTATE mom_kin_4th_axis_vector $rot0 Z V3

   VECTOR_ROTATE mom_kin_5th_axis_vector $rot1 V1 X
   VECTOR_ROTATE mom_kin_5th_axis_vector $rot1 V2 Y
   VECTOR_ROTATE mom_kin_5th_axis_vector $rot1 V3 Z

   MTX3_init_x_y_z X Y Z matrix

   return 1
}


#=============================================================
proc DPP_GE_COOR_ROT_AUTO3D_WCS_ROTATION { first_vec second_vec angle coord_offset rot_pos } {
#=============================================================
# This procedure is used to detect if an operation is 3+2 operation without Local CSYS rotation coordinate system. And calculate the
# parameters for G68.
#
# Output:
#   first_vec - first vector for G68.
#   second_vec - second vector for G68.
#   angle - the angles rotate around first_vec and second_vec.
#   coord_offset - the linear offset for G68.
#   rot_pos    - current position respect to rotated coordinate system
#
# Return:
#   1 - operation is 3+2 operation without Local CSYS rotation coordinate system.
#   0 - operation is not 3+2 operation without Local CSYS rotation coordinate system.
#
# Revisions:
#-----------
# 2013-05-25 levi - Initial implementation
#
   upvar $first_vec g68_first_vec
   upvar $second_vec g68_second_vec
   upvar $angle g68_coord_rotation
   upvar $coord_offset offset
   upvar $rot_pos pos

   global mom_kin_coordinate_system_type
   global mom_kin_machine_type
   global mom_kin_4th_axis_point mom_kin_5th_axis_point
   global mom_kin_4th_axis_vector mom_kin_5th_axis_vector
   global mom_out_angle_pos mom_prev_out_angle_pos
   global DEG2RAD RAD2DEG
   global mom_pos mom_mcs_goto mom_prev_pos
   global dpp_ge
   global save_mom_kin_machine_type
   global save_mom_kin_4th_axis_vector

   set v0 0.0; set v1 1.0
   VEC3_init v1 v0 v0 X
   VEC3_init v0 v1 v0 Y
   VEC3_init v0 v0 v1 Z
   MTX3_init_x_y_z X Y Z matrix
   VMOV 3 mom_pos pos

   if { ![string match "*5_axis*" $mom_kin_machine_type] } {
      return 0
   }

   if { [info exists mom_kin_coordinate_system_type] && ![string compare "CSYS" $mom_kin_coordinate_system_type] } {
      return 0
   } else {
      if { (![EQ_is_zero $mom_out_angle_pos(0)] && ![VEC3_is_equal mom_kin_4th_axis_vector Z]) || \
           (![EQ_is_zero $mom_out_angle_pos(1)] && ![VEC3_is_equal mom_kin_5th_axis_vector Z]) } {
      } else {
         return 0
      }
   }

  # Save kinematics
   DPP_GE_SAVE_KINEMATICS

  # get rotation angle
   if { [string match "5_axis_dual_head" $mom_kin_machine_type] } {

     # Swap rotary axes kinematics for dual head machine
      DPP_GE_SWAP_4TH_5TH_KINEMATICS

      set ang_pos(0) $mom_out_angle_pos(1)
      set ang_pos(1) $mom_out_angle_pos(0)

     # Swap rotary axes value due to kinemtics switched
      set mom_out_angle_pos(0) $ang_pos(0)
      set mom_out_angle_pos(1) $ang_pos(1)
      set mom_prev_out_angle_pos(0) $ang_pos(0)
      set mom_prev_out_angle_pos(1) $ang_pos(1)

     #<Nov-27-2018 gsl>
     if 0 {
      set mom_pos(3) $ang_pos(0)
      set mom_pos(4) $ang_pos(1)
      set mom_prev_pos(3) $ang_pos(0)
      set mom_prev_pos(4) $ang_pos(1)
     } else {
      set temp $mom_pos(3)
      set mom_pos(3) $mom_pos(4)
      set mom_pos(4) $temp
     # set mom_prev_pos(3) $mom_pos(3)
     # set mom_prev_pos(4) $mom_pos(4)
      set temp $mom_prev_pos(3)
      set mom_prev_pos(3) $mom_prev_pos(4)
      set mom_prev_pos(4) $temp
     }
      MOM_reload_variable -a mom_out_angle_pos
      MOM_reload_variable -a mom_prev_out_angle_pos
      MOM_reload_variable -a mom_pos
      MOM_reload_variable -a mom_prev_pos
   } else {
      set ang_pos(0) $mom_out_angle_pos(0)
      set ang_pos(1) $mom_out_angle_pos(1)
   }

   set rot0 [expr $ang_pos(0)*$DEG2RAD]
   set rot1 [expr $ang_pos(1)*$DEG2RAD]

  # Reolad kinematics to dual-table machine
   if { ![string match "5_axis_dual_table" $mom_kin_machine_type] } {
      set mom_kin_machine_type "5_axis_dual_table"
   }

   set v0 0.0
   VEC3_init v0 v0 v0 mom_kin_4th_axis_point
   VEC3_init v0 v0 v0 mom_kin_5th_axis_point
   MOM_reload_kinematics

  # Get current position respect to rotated coordinate
   VECTOR_ROTATE mom_kin_5th_axis_vector [expr -1*$rot1] mom_mcs_goto V
   VECTOR_ROTATE mom_kin_4th_axis_vector [expr -1*$rot0] V pos

  # Calculate rotation matrix
   VECTOR_ROTATE mom_kin_4th_axis_vector $rot0 X V1
   VECTOR_ROTATE mom_kin_4th_axis_vector $rot0 Y V2
   VECTOR_ROTATE mom_kin_4th_axis_vector $rot0 Z V3

   VECTOR_ROTATE mom_kin_5th_axis_vector $rot1 V1 X
   VECTOR_ROTATE mom_kin_5th_axis_vector $rot1 V2 Y
   VECTOR_ROTATE mom_kin_5th_axis_vector $rot1 V3 Z

   MTX3_init_x_y_z X Y Z matrix

   # extend the matrix between fixture offset and dummy local csys to 4X4 matrix
    for {set i 0} {$i<3} {incr i} {
       set extend_matrix($i) $matrix($i)
    }
    set extend_matrix(3) 0
    for {set i 4} {$i<7} {incr i} {
       set extend_matrix($i) $matrix([expr $i-1])
    }
    set extend_matrix(7) 0
    for {set i 8} {$i<11} {incr i} {
       set extend_matrix($i) $matrix([expr $i-2])
    }
    set extend_matrix(11) 0
    for {set i 12} {$i<15} {incr i} {
       set extend_matrix($i) 0
    }
    set extend_matrix(15) 1

   # calculate the parameters for G68 including linear offset, vectors and rotation angles
    CALCULATE_G68 "AUTO_3D" extend_matrix offset g68_first_vec g68_coord_rotation
    set g68_second_vec(0) 0
    set g68_second_vec(1) 0
    set g68_second_vec(2) 1

   # if it's a head-table machine, recalculate the vectors and angles to just output G68 once
    if { $save_mom_kin_machine_type == "5_axis_head_table" } {
       VMOV 3 save_mom_kin_4th_axis_vector g68_first_vec
       set g68_coord_rotation(0) $mom_pos(3)
       set g68_coord_rotation(1) 0
       set g68_coord_rotation(2) 0
    }

   # if it's a dual_head machine, recalculate the vectors and angles to output G68 aroud the rotary axis vectors
    if { $save_mom_kin_machine_type == "5_axis_dual_head" } {
       VMOV 3 mom_kin_5th_axis_vector g68_first_vec
       VMOV 3 mom_kin_4th_axis_vector g68_second_vec
       set g68_coord_rotation(0) [expr $rot1*$RAD2DEG]
       set g68_coord_rotation(1) [expr $rot0*$RAD2DEG]
       set g68_coord_rotation(2) 0
    }

    return 1
}


#=============================================================
proc DPP_GE_COOR_ROT_LOCAL { rot_matrix coord_offset } {
#=============================================================
# This procedure is used to detect if an operation is under a local CSYS rotation and if the coordinate is rotated.
# It will return rotation matrix.
#
# Output:
#   rot_matrix - rotation matrix, mapping from the current local CSYS rotation coordinate to parent coordinate.
#   coord_offset - linear coordinate offset from current local CSYS rotation coordinate to parent coordinate.
#
# Return:
#   1 - operation is under local CSYS rotation coordinate system and the coordinate is rotated.
#   0 - operation is not under local CSYS rotation coordinate system or the coordinate is not rotated.
#
# Revisions:
#-----------
# 2013-05-22 lili - Initial implementation
#
   upvar $rot_matrix matrix
   upvar $coord_offset offset

   global mom_csys_matrix mom_csys_origin
   global mom_kin_coordinate_system_type
   global mom_parent_csys_matrix
   global mom_part_unit mom_output_unit

   set v0 0; set v1 1
   VEC3_init v1 v0 v0 VX
   VEC3_init v0 v1 v0 VY
   VEC3_init v0 v0 v1 VZ
   MTX3_init_x_y_z VX VY VZ matrix
   MTX3_init_x_y_z VX VY VZ rr_matrix

   if { [info exists mom_kin_coordinate_system_type] && ![string compare "CSYS" $mom_kin_coordinate_system_type] } {
      if { [array exists mom_parent_csys_matrix] } {
         VMOV 9 mom_parent_csys_matrix matrix

         if { ![string compare $mom_part_unit $mom_output_unit] } {
            set unit_conversion 1
         } elseif { ![string compare "IN" $mom_output_unit] } {
            set unit_conversion [expr 1.0/25.4]
         } else {
            set unit_conversion 25.4
         }
         set offset(0) [expr $unit_conversion*$mom_parent_csys_matrix(9)]
         set offset(1) [expr $unit_conversion*$mom_parent_csys_matrix(10)]
         set offset(2) [expr $unit_conversion*$mom_parent_csys_matrix(11)]

      } else {
         VMOV 9 mom_csys_matrix matrix
         VMOV 3 mom_csys_origin offset
      }
      if { [MTX3_is_equal matrix rr_matrix] } {
         return 0
      } else {
         return 1
      }
   } else {
      return 0
   }
}


#=============================================================
proc DPP_GE_COOR_ROT_LOCAL_WCS_ROTATION { first_vec second_vec angle coord_offset } {
#=============================================================
# This procedure is used to calculate the parameters for G68 under local csys condition.
#
# Output:
#   first_vec - first vector for G68.
#   second_vec - second vector for G68.
#   angle - the angles rotate around first_vec and second_vec.
#   coord_offset - the linear offset for G68.
#
# Return:
#   1 - operation is under local CSYS rotation coordinate system and the coordinate is rotated.
#   0 - operation is not under local CSYS rotation coordinate system or the coordinate is not rotated.
#
# Revisions:
#-----------
# 2013-05-25 levi - Initial implementation
#
   upvar $first_vec g68_first_vec
   upvar $second_vec g68_second_vec
   upvar $angle g68_coord_rotation
   upvar $coord_offset offset

   global mom_csys_matrix mom_csys_origin
   global mom_kin_coordinate_system_type
   global mom_parent_csys_matrix
   global mom_part_unit mom_output_unit

   set v0 0; set v1 1
   VEC3_init v1 v0 v0 VX
   VEC3_init v0 v1 v0 VY
   VEC3_init v0 v0 v1 VZ
   MTX3_init_x_y_z VX VY VZ matrix
   MTX3_init_x_y_z VX VY VZ rr_matrix

   if { [info exists mom_kin_coordinate_system_type] && ![string compare "CSYS" $mom_kin_coordinate_system_type] } {
      if { [array exists mom_parent_csys_matrix] } {
         VMOV 9 mom_parent_csys_matrix matrix

         if { ![string compare $mom_part_unit $mom_output_unit] } {
            set unit_conversion 1
         } elseif { ![string compare "IN" $mom_output_unit] } {
            set unit_conversion [expr 1.0/25.4]
         } else {
            set unit_conversion 25.4
         }
         set offset(0) [expr $unit_conversion*$mom_parent_csys_matrix(9)]
         set offset(1) [expr $unit_conversion*$mom_parent_csys_matrix(10)]
         set offset(2) [expr $unit_conversion*$mom_parent_csys_matrix(11)]
         set mom_parent_csys_matrix(9) $offset(0)
         set mom_parent_csys_matrix(10) $offset(1)
         set mom_parent_csys_matrix(11) $offset(2)

      } else {
         VMOV 9 mom_csys_matrix matrix
         VMOV 3 mom_csys_origin offset
      }
      if { [MTX3_is_equal matrix rr_matrix] } {
         return 0
      } else {
         # extend the matrix between fixture offset and dummy local csys to 4X4 matrix
         for { set i 0 } { $i < 3 } { incr i } {
            set extend_matrix($i) $matrix($i)
         }
         set extend_matrix(3) 0
         for { set i 4 } { $i < 7 } { incr i } {
            set extend_matrix($i) $matrix([expr $i-1])
         }
         set extend_matrix(7) 0
         for { set i 8 } { $i < 11 } { incr i } {
            set extend_matrix($i) $matrix([expr $i-2])
         }
         set extend_matrix(11) 0
         for { set i 12 } { $i < 15 } { incr i } {
            set extend_matrix($i) $offset([expr $i-12])
         }
         set extend_matrix(15) 1
         CALCULATE_G68 "LOCAL" extend_matrix offset g68_first_vec g68_coord_rotation
         set g68_second_vec(0) 0
         set g68_second_vec(1) 0
         set g68_second_vec(2) 1
         return 1
      }
   } else {
      return 0
   }
}


#=============================================================
proc DPP_GE_COOR_ROT_WCS_ROTATION { g68_first_vec g68_second_vec g68_coord_rotation offset pos } {
#=============================================================
# This procedure is used to detect if operation has coordinate rotation, and calculate the parameters for G68.
# DPP_GE_COOR_ROT_LOCAL_WCS_ROTATION, DPP_GE_COOR_ROT_AUTO3D_WCS_ROTATION are called in this proc.
#
# Input:
#
#
# Output:
#   g68_first_vec - first vector for G68.
#   g68_second_vec - second vector for G68.
#   g68_coord_rotation - the angles rotate around first_vec and second_vec.
#   offset - the linear offset for G68.
#   pos - linear axes position respect to rotated cooridnate
#
# Return:
#   detected coordinate mode. Possible value: NONE, LOCAL, AUTO_3D
#   NONE - no coordinate rotation
#   LOCAL - coordinate rotation set up by LOCAL CSYS MCS
#   AUTO_3D - coordinate rotation set up by tilt work plane
#
# Revisions:
#-----------
# 2013-05-25 levi - Initial implementation
###

   upvar $g68_first_vec first_vec
   upvar $g68_second_vec second_vec
   upvar $g68_coord_rotation angle
   upvar $offset coord_offset
   upvar $pos rot_pos

   global mom_pos
   VMOV 3 mom_pos rot_pos

   set v0 0
   VEC3_init v0 v0 v0 first_vec
   VEC3_init v0 v0 v0 second_vec
   VEC3_init v0 v0 v0 angle
   VEC3_init v0 v0 v0 coord_offset

   if { [DPP_GE_COOR_ROT_LOCAL_WCS_ROTATION first_vec second_vec angle coord_offset] } {
      set coord_rot "LOCAL"
   } elseif { [DPP_GE_COOR_ROT_AUTO3D_WCS_ROTATION first_vec second_vec angle coord_offset rot_pos] } {
      set coord_rot "AUTO_3D"
   } else {
      set coord_rot "NONE"
   }

   if { [string match "NONE" $coord_rot] } {
      set angle(0) 0.0
      set angle(1) 0.0
      set angle(2) 0.0
   }

   return $coord_rot
}


#=============================================================
proc DPP_GE_DEBUG { args } {
#=============================================================
# This procedure is used to debug.
#
#<12-03-2012 Allen> - Initial version
   foreach dpp_input_var  $args {
      upvar $dpp_input_var  dpp_output_var
      MOM_output_to_listing_device " [format "%-30s  %-40s %-30s " $dpp_input_var  $dpp_output_var [info level [expr [info level]-1]] ]"
   }
}


#=============================================================
proc DPP_GE_DETECT_5AXIS_TOOL_PATH { } {
#=============================================================
# This procedure is used to detect the if an operation is a 5-axis simultaneous milling operation.
# In this command, tool path type is detected by mom_operation_type, mom_tool_path_type and mom_tool_axis_type
# The result may not always match 5-axis simultaneous milling. It is more tolerance.
#
# Return:
#   1 - tool path is 5 axis simultaneous
#   0 - tool path is not 5 axis simultaneous
#
# Revisions:
#-----------
# 2013-05-22 lili - Initial implementation
#

   global mom_tool_axis_type
   global mom_tool_path_type
   global mom_operation_type

   if { ![info exists mom_tool_axis_type] } {
      set mom_tool_axis_type 0
   }
   if { ![info exists mom_tool_path_type] } {
      set mom_tool_path_type "undefined"
   }

   if { [DPP_GE_DETECT_HOLE_CUTTING_OPERATION] } {
      return 0
   } elseif { ($mom_tool_axis_type >= 2 && [string match "Variable-axis *" $mom_operation_type]) ||\
              ![string compare "Sequential Mill Main Operation" $mom_operation_type] || \
              (![string compare "variable_axis" $mom_tool_path_type] && ![string match "Variable-axis *" $mom_operation_type]) } {
     return 1
  } else {
     return 0
  }
}


#=============================================================
proc DPP_GE_DETECT_HOLE_CUTTING_OPERATION { } {
#=============================================================
# This procedure is used to detect if the operation is a hole cutting operation.
# Hole cutting operation includs Cylinder Milling, Thread Milling, Point to Point,
# Hole Making, Drilling
#
# Return:
#   1 - operation is hole cutting operation
#   0 - operation is not hole cutting operation
#
# Revisions:
#-----------
# 2013-05-22 lili - Initial implementation
# 2015-04-22 Jintao - PR7281995 add Chamfer Milling and Radial Groove Milling
# 2017-08-31 Toon VanderKooi PR8958320 add Generic Feature Operation
#

   global mom_operation_type

   if { ![string compare "Hole Making"               $mom_operation_type] ||\
        ![string compare "Point to Point"            $mom_operation_type] ||\
        ![string compare "Cylinder Milling"          $mom_operation_type] ||\
        ![string compare "Thread Milling"            $mom_operation_type] ||\
        ![string compare "Drilling"                  $mom_operation_type] ||\
        ![string compare "Chamfer Milling"           $mom_operation_type] ||\
        ![string compare "Generic Feature Operation" $mom_operation_type] ||\
        ![string compare "Radial Groove Milling"     $mom_operation_type] } {
      return 1
   } else {
      return 0
   }
}


#=============================================================
proc DPP_GE_DETECT_TOOL_PATH_TYPE { } {
#=============================================================
# This procedure is used to set dpp_ge(toolpath_axis_num)

   global dpp_ge
   if { [DPP_GE_DETECT_5AXIS_TOOL_PATH] } {
      set dpp_ge(toolpath_axis_num) 5
   } else {
      set dpp_ge(toolpath_axis_num) 3
   }
}


#=============================================================
proc DPP_GE_GET_NCM_WORK_PLANE_CHANGE_MODE { } {
#=============================================================
# This proc is used to detect current motion work plane change mode.
# Return:
# dpp_ge(ncm_work_plane_change_mode)
#
# "direct_change" - Current motion is non-cutting; work plane has changed. There's only one non-cuttong motion between current work plane to next.
# "start_change"  - Current motion is first work plane change NCM motion. There are more than one non-cutting motion between current work plane to next.
# "in_change"     - Current motion is non-cutting. There are more than one non-cutting motion between two different work planes. This motion is not the first NCM.
# "end_change"    - Current motion is last ncm motion between two different work plane.
# None            - It is either cutting motion or initial/first move; or there is no work plane change
#
# Revisions:
#-----------
# 2016-12-22  lili - initial version
  global mom_current_motion
  global mom_motion_type mom_nxt_motion_type
  global mom_tool_axis mom_prev_tool_axis
  global dpp_ge

  if { ![string compare $mom_current_motion "initial_move"] || ![string compare $mom_current_motion "first_move"] } {
    #for safe , reset it at initial move and first move
     set dpp_ge(ncm_work_plane_change_mode) "None"
  }

  if { [string compare "RAPID" $mom_motion_type] && [string compare "TRAVERSAL" $mom_motion_type] } {
     # current motion is cutting motion
     set dpp_ge(ncm_work_plane_change_mode) "None"
  } elseif {[info exists mom_nxt_motion_type] && [info exists mom_prev_tool_axis]} {
    # current motion is non-cutting motion
     if { [info exists dpp_ge(ncm_work_plane_change_mode)] && \
          ($dpp_ge(ncm_work_plane_change_mode) == "start_change" || $dpp_ge(ncm_work_plane_change_mode) == "in_change") } {
        if { [string compare "RAPID" $mom_nxt_motion_type] && [string compare "TRAVERSAL" $mom_nxt_motion_type] } {
           # it is actually traverse end
           set dpp_ge(ncm_work_plane_change_mode) "end_change"
        } else {
           set dpp_ge(ncm_work_plane_change_mode) "in_change"
        }
     } elseif { ![VEC3_is_equal mom_tool_axis mom_prev_tool_axis] } {
        if { ![string compare "RAPID" $mom_nxt_motion_type] || ![string compare "TRAVERSAL" $mom_nxt_motion_type] } {
           set dpp_ge(ncm_work_plane_change_mode) "start_change"
        } else {
           set dpp_ge(ncm_work_plane_change_mode) "direct_change"
        }
     } else {
        set dpp_ge(ncm_work_plane_change_mode) "None"
     }
  } else {
     set dpp_ge(ncm_work_plane_change_mode) "None"
  }
}


#=============================================================
proc DPP_GE_RESTORE_KINEMATICS { } {
#=============================================================
# This procedure is used to restore original kinematics variables and sys variables.
#
# 10-16-2013 levi - Exchange the pos of 4th axis and 5th axis when restore kinematics for dual head machine.
# 09-16-2015 szl  - Added save_kin_machine_type exist check.
# Jul-11-2018 gsl - Revised to account for linked posts
# Nov-27-2018 gsl - Revised to set prev state properly
#
   global save_mom_kin_machine_type
   global mom_kin_machine_type
   global mom_out_angle_pos
   global mom_pos
   global mom_prev_out_angle_pos
   global mom_prev_pos

   if { ![info exists save_mom_kin_machine_type] } {
      return
   }

  # If it's dual-head machine, exchange the angle pos for 4th axis and 5th axis for the first point after auto3d
   if { [string match "5_axis_dual_head" $save_mom_kin_machine_type] && \
       ![string match $save_mom_kin_machine_type $mom_kin_machine_type] } {

      set temp $mom_out_angle_pos(0)
      set mom_out_angle_pos(0) $mom_out_angle_pos(1)
      set mom_out_angle_pos(1) $temp

     #<Nov-27-2018 gsl> Must fix - Do not override prev angle
     # VMOV 2 mom_out_angle_pos mom_prev_out_angle_pos
      set temp $mom_prev_out_angle_pos(0)
      set mom_prev_out_angle_pos(0) $mom_prev_out_angle_pos(1)
      set mom_prev_out_angle_pos(1) $temp

     #<Nov-27-2018 gsl>
     if 0 { ;# MUST FIX -
      set mom_pos(3) $mom_out_angle_pos(0)
      set mom_pos(4) $mom_out_angle_pos(1)
      set mom_prev_pos(3) $mom_pos(3)
      set mom_prev_pos(4) $mom_pos(4)
     } else {
      set temp $mom_pos(3)
      set mom_pos(3) $mom_pos(4)
      set mom_pos(4) $temp
      set mom_prev_pos(3) $mom_pos(3)
      set mom_prev_pos(4) $mom_pos(4)
     }

      MOM_reload_variable -a mom_out_angle_pos
      MOM_reload_variable -a mom_prev_out_angle_pos
      MOM_reload_variable -a mom_pos
      MOM_reload_variable -a mom_prev_pos
   }

   set kin_list { mom_sys_4th_axis_has_limits   mom_sys_5th_axis_has_limits  mom_kin_machine_type \
                  mom_kin_4th_axis_ang_offset   mom_kin_arc_output_mode      mom_kin_4th_axis_direction \
                  mom_kin_4th_axis_incr_switch  mom_kin_4th_axis_leader      mom_kin_4th_axis_limit_action \
                  mom_kin_4th_axis_max_limit    mom_kin_4th_axis_min_incr    mom_kin_4th_axis_min_limit \
                  mom_kin_4th_axis_plane        mom_kin_4th_axis_rotation    mom_kin_4th_axis_type \
                  mom_kin_5th_axis_zero         mom_kin_4th_axis_zero        mom_kin_5th_axis_direction \
                  mom_kin_5th_axis_incr_switch  mom_kin_5th_axis_leader      mom_kin_5th_axis_limit_action \
                  mom_kin_5th_axis_max_limit    mom_kin_5th_axis_min_incr    mom_kin_5th_axis_min_limit \
                  mom_kin_5th_axis_plane        mom_kin_5th_axis_rotation    mom_kin_5th_axis_type \
                  mom_kin_5th_axis_ang_offset   mom_kin_helical_arc_output_mode }

   set kin_array_list { mom_kin_4th_axis_center_offset  mom_kin_5th_axis_center_offset   mom_kin_4th_axis_point \
                        mom_kin_5th_axis_point          mom_kin_4th_axis_vector          mom_kin_5th_axis_vector \
                        mom_kin_spindle_axis }


  # CURRENT_HEAD will be set when a linked post is brought in via HEAD UDE (MOM_head).
  #
   if { [info exists ::CURRENT_HEAD] && $::CURRENT_HEAD != "" && [info exists ::mom_sys_postname($::CURRENT_HEAD)] } {

      set CURR_POST "$::mom_sys_postname($::CURRENT_HEAD)"

      foreach kin_var $kin_list {
         global $kin_var
         if { [info exists ${CURR_POST}::save_$kin_var] } {
            set $kin_var [set ${CURR_POST}::save_$kin_var]
         }
      }

      foreach kin_var $kin_array_list {
         global $kin_var
         if { [array exists ${CURR_POST}::save_$kin_var] } {
            set save_var ${CURR_POST}::save_$kin_var
            VMOV 3 $save_var $kin_var
         }
      }

      global mom_sys_leader
      if { [info exists ${CURR_POST}::save_mom_sys_leader(fourth_axis_home)] } {
         set mom_sys_leader(fourth_axis_home) [set ${CURR_POST}::save_mom_sys_leader(fourth_axis_home)]
      }
      if { [info exists ${CURR_POST}::save_mom_sys_leader(fifth_axis_home)] } {
         set mom_sys_leader(fifth_axis_home)  [set ${CURR_POST}::save_mom_sys_leader(fifth_axis_home)]
      }

   } else {

      foreach kin_var $kin_list {
         global $kin_var save_$kin_var
         if { [info exists save_$kin_var] } {
            set $kin_var [set save_$kin_var]
         }
      }

      foreach kin_var $kin_array_list {
         global $kin_var save_$kin_var
         if { [array exists save_$kin_var] } {
            set save_var save_$kin_var
            VMOV 3 $save_var $kin_var
         }
      }

      global mom_sys_leader save_mom_sys_leader
      if { [info exists save_mom_sys_leader(fourth_axis_home)] } {
         set mom_sys_leader(fourth_axis_home) $save_mom_sys_leader(fourth_axis_home)
      }
      if { [info exists save_mom_sys_leader(fifth_axis_home)] } {
         set mom_sys_leader(fifth_axis_home) $save_mom_sys_leader(fifth_axis_home)
      }
   }

   global mom_sys_leader
   if { [info exists mom_kin_4th_axis_leader] } {
      set mom_sys_leader(fourth_axis) $mom_kin_4th_axis_leader
   }
   if { [info exists mom_kin_5th_axis_leader] } {
      set mom_sys_leader(fifth_axis) $mom_kin_5th_axis_leader
   }

   MOM_reload_kinematics
}


#=============================================================
proc DPP_GE_SAVE_KINEMATICS { } {
#=============================================================
# This procedure is used to save original kinematics variables
#
# Jul-10-2018 gsl - To accommodate linked posts situation
# Oct-23-2018 gsl - Fixed problem where ::save_mom_kin_machine_type is not saved in linked post setting.
#
   set kin_list { mom_sys_4th_axis_has_limits   mom_sys_5th_axis_has_limits  mom_kin_machine_type \
                  mom_kin_4th_axis_ang_offset   mom_kin_arc_output_mode      mom_kin_4th_axis_direction \
                  mom_kin_4th_axis_incr_switch  mom_kin_4th_axis_leader      mom_kin_4th_axis_limit_action \
                  mom_kin_4th_axis_max_limit    mom_kin_4th_axis_min_incr    mom_kin_4th_axis_min_limit \
                  mom_kin_4th_axis_plane        mom_kin_4th_axis_rotation    mom_kin_4th_axis_type \
                  mom_kin_5th_axis_zero         mom_kin_4th_axis_zero        mom_kin_5th_axis_direction \
                  mom_kin_5th_axis_incr_switch  mom_kin_5th_axis_leader      mom_kin_5th_axis_limit_action \
                  mom_kin_5th_axis_max_limit    mom_kin_5th_axis_min_incr    mom_kin_5th_axis_min_limit \
                  mom_kin_5th_axis_plane        mom_kin_5th_axis_rotation    mom_kin_5th_axis_type \
                  mom_kin_5th_axis_ang_offset   mom_kin_helical_arc_output_mode }

   set kin_array_list { mom_kin_4th_axis_center_offset  mom_kin_5th_axis_center_offset   mom_kin_4th_axis_point \
                        mom_kin_5th_axis_point          mom_kin_4th_axis_vector          mom_kin_5th_axis_vector }


  # CURRENT_HEAD will be set when a linked post is brought in via HEAD UDE (MOM_head).
  #
   if { [info exists ::CURRENT_HEAD] && $::CURRENT_HEAD != "" && [info exists ::mom_sys_postname($::CURRENT_HEAD)] } {

      set CURR_POST "$::mom_sys_postname($::CURRENT_HEAD)"

      if { ![namespace exists $CURR_POST] } {
         namespace eval $CURR_POST {}
      }

      foreach kin_var $kin_list {
         global $kin_var
         if { [info exists $kin_var] && ![info exists ${CURR_POST}::save_$kin_var] } {
            set ${CURR_POST}::save_$kin_var [set $kin_var]
         }

        #<10/23/2018 gsl> Special treatment for next var which is referenced to restore Kin vars properly.
         if { [string match "mom_kin_machine_type" $kin_var] } {
            set ::save_$kin_var [set $kin_var]
         }
      }

      foreach kin_var $kin_array_list {
         global $kin_var
         if { [array exists $kin_var] && ![array exists ${CURR_POST}::save_$kin_var] } {
            set save_var ${CURR_POST}::save_$kin_var
            VMOV 3 $kin_var $save_var
         }
      }

      global mom_sys_leader
      if { [info exists mom_sys_leader(fourth_axis_home)] && ![info exists ${CURR_POST}::save_mom_sys_leader(fourth_axis_home)] } {
         set ${CURR_POST}::save_mom_sys_leader(fourth_axis_home) $mom_sys_leader(fourth_axis_home)
      }
      if { [info exists mom_sys_leader(fifth_axis_home)]  && ![info exists ${CURR_POST}::save_mom_sys_leader(fifth_axis_home)] } {
         set ${CURR_POST}::save_mom_sys_leader(fifth_axis_home) $mom_sys_leader(fifth_axis_home)
      }

   } else {

      foreach kin_var $kin_list {
         global $kin_var save_$kin_var
         if { [info exists $kin_var] && ![info exists save_$kin_var] } {
            set save_$kin_var [set $kin_var]
         }
      }

      foreach kin_var $kin_array_list {
         global $kin_var save_$kin_var
         if { [array exists $kin_var] && ![array exists save_$kin_var] } {
            set save_var save_$kin_var
            VMOV 3 $kin_var $save_var
         }
      }

      global mom_sys_leader save_mom_sys_leader
      if { [info exists mom_sys_leader(fourth_axis_home)] && ![info exists save_mom_sys_leader(fourth_axis_home)] } {
         set save_mom_sys_leader(fourth_axis_home) $mom_sys_leader(fourth_axis_home)
      }
      if { [info exists mom_sys_leader(fifth_axis_home)] && ![info exists save_mom_sys_leader(fifth_axis_home)] } {
         set save_mom_sys_leader(fifth_axis_home) $mom_sys_leader(fifth_axis_home)
      }
   }
}


#=============================================================
proc DPP_GE_SWAP_4TH_5TH_KINEMATICS { } {
#=============================================================
# This procedure is used to swap 4th and 5th axis kinematics variables
#
  set kin_list { ang_offset   direction leader  incr_switch  \
                 limit_action max_limit min_incr min_limit \
                 plane        rotation  zero}

  set kin_array_list { center_offset point vector}

 foreach kin_var $kin_list {
    global mom_kin_4th_axis_$kin_var save_mom_kin_4th_axis_$kin_var
    global mom_kin_5th_axis_$kin_var save_mom_kin_5th_axis_$kin_var
    if {[info exists save_mom_kin_4th_axis_$kin_var] && [info exists save_mom_kin_5th_axis_$kin_var]} {
       set mom_kin_4th_axis_$kin_var [set save_mom_kin_5th_axis_[set kin_var]]
       set mom_kin_5th_axis_$kin_var [set save_mom_kin_4th_axis_[set kin_var]]
    }
 }

 foreach kin_var $kin_array_list {
    global mom_kin_4th_axis_$kin_var save_mom_kin_4th_axis_$kin_var
    global mom_kin_5th_axis_$kin_var save_mom_kin_5th_axis_$kin_var
    if {[array exists save_mom_kin_4th_axis_$kin_var] && [array exists save_mom_kin_5th_axis_$kin_var]} {
       VMOV 3 save_mom_kin_4th_axis_$kin_var mom_kin_5th_axis_$kin_var
       VMOV 3 save_mom_kin_5th_axis_$kin_var mom_kin_4th_axis_$kin_var
    }
 }

 global mom_sys_4th_axis_has_limits save_mom_sys_5th_axis_has_limits
 global mom_sys_5th_axis_has_limits save_mom_sys_4th_axis_has_limits
 global mom_sys_leader save_mom_sys_leader
 if {[info exists save_mom_sys_4th_axis_has_limits] && [info exists save_mom_sys_5th_axis_has_limits]} {
    set mom_sys_4th_axis_has_limits $save_mom_sys_5th_axis_has_limits
    set mom_sys_5th_axis_has_limits $save_mom_sys_4th_axis_has_limits
 }
 if {[info exists save_mom_kin_4th_axis_leader] && [info exists save_mom_kin_5th_axis_leader]} {
    set mom_sys_leader(fourth_axis) $save_mom_kin_5th_axis_leader
    set mom_sys_leader(fifth_axis) $save_mom_kin_4th_axis_leader
 }
 if {[info exists save_mom_sys_leader(fourth_axis_home)] && [info exists save_mom_sys_leader(fifth_axis_home)]} {
    set mom_sys_leader(fourth_axis_home) $save_mom_sys_leader(fifth_axis_home)
    set mom_sys_leader(fifth_axis_home)  $save_mom_sys_leader(fourth_axis_home)
 }

 MOM_reload_kinematics
}


#=============================================================
proc DPP_GE_UNSET_KINEMATICS { } {
#=============================================================
#This proc is used to unset saved kinematics variables

  set kin_list { mom_sys_4th_axis_has_limits   mom_sys_5th_axis_has_limits  mom_kin_machine_type \
                 mom_kin_4th_axis_ang_offset   mom_kin_arc_output_mode      mom_kin_4th_axis_direction \
                 mom_kin_4th_axis_incr_switch  mom_kin_4th_axis_leader      mom_kin_4th_axis_limit_action \
                 mom_kin_4th_axis_max_limit    mom_kin_4th_axis_min_incr    mom_kin_4th_axis_min_limit \
                 mom_kin_4th_axis_plane        mom_kin_4th_axis_rotation    mom_kin_4th_axis_type \
                 mom_kin_5th_axis_zero         mom_kin_4th_axis_zero        mom_kin_5th_axis_direction \
                 mom_kin_5th_axis_incr_switch  mom_kin_5th_axis_leader      mom_kin_5th_axis_limit_action \
                 mom_kin_5th_axis_max_limit    mom_kin_5th_axis_min_incr    mom_kin_5th_axis_min_limit \
                 mom_kin_5th_axis_plane        mom_kin_5th_axis_rotation    mom_kin_5th_axis_type \
                 mom_kin_5th_axis_ang_offset   mom_kin_helical_arc_output_mode }

  set kin_array_list { mom_kin_4th_axis_center_offset  mom_kin_5th_axis_center_offset   mom_kin_4th_axis_point \
                       mom_kin_5th_axis_point          mom_kin_4th_axis_vector          mom_kin_5th_axis_vector }


  foreach kin_var $kin_list {
    global save_$kin_var
    if {[info exists save_$kin_var]} {
       unset save_$kin_var
    }
  }

  foreach kin_var $kin_array_list {
    global save_$kin_var
    if [array exists save_$kin_var] {
      UNSET_VARS save_$kin_var
    }
  }

  global mom_sys_leader save_mom_sys_leader
  if {[info exists save_mom_sys_leader(fourth_axis_home)]} {
     unset save_mom_sys_leader(fourth_axis_home)
  }
  if {[info exists save_mom_sys_leader(fifth_axis_home)]} {
     unset save_mom_sys_leader(fifth_axis_home)
  }
}


#=============================================================
proc EXEC { command_string {__wait 1} } {
#=============================================================
# This command can be used in place of the intrinsic Tcl "exec" command
# of which some problems have been reported under Win64 O/S and multi-core
# processors environment.
#
#
# Input:
#   command_string -- command string
#   __wait         -- optional flag
#                     1 (default)   = Caller will wait until execution is complete.
#                     0 (specified) = Caller will not wait.
#
# Return:
#   Results of execution
#
#
# Revisions:
#-----------
# 05-19-10 gsl - Initial implementation
#

   global tcl_platform


   if { $__wait } {

      if { [string match "windows" $tcl_platform(platform)] } {

         global env mom_logname

        # Create a temporary file to collect output
         set result_file "$env(TEMP)/${mom_logname}__EXEC_[clock clicks].out"

        # Clean up existing file
         regsub -all {\\} $result_file {/}  result_file
        #regsub -all { }  $result_file {\ } result_file

         if { [file exists "$result_file"] } {
            file delete -force "$result_file"
         }

        #<11-05-2013> Escape spaces
         set cmd [concat exec $command_string > \"$result_file\"]
         regsub -all {\\} $cmd {\\\\} cmd
         regsub -all { }  $result_file {\\\ } result_file

         eval $cmd

        # Return results & clean up temporary file
         if { [file exists "$result_file"] } {
            set fid [open "$result_file" r]
            set result [read $fid]
            close $fid

            file delete -force "$result_file"

           return $result
         }

      } else {

         set cmd [concat exec $command_string]

        return [eval $cmd]
      }

   } else {

      if { [string match "windows" $tcl_platform(platform)] } {

         set cmd [concat exec $command_string &]
         regsub -all {\\} $cmd {\\\\} cmd

        return [eval $cmd]

      } else {

        return [exec $command_string &]
      }
   }
}




#=============================================================
proc GET_SPINDLE_AXIS { input_tool_axis } {
#=============================================================
# called by ROTARY_AXIS_RETRACT

   upvar $input_tool_axis axis

   global mom_kin_4th_axis_type
   global mom_kin_4th_axis_plane
   global mom_kin_5th_axis_type
   global mom_kin_spindle_axis
   global mom_sys_spindle_axis

   if { ![string compare "Table" $mom_kin_4th_axis_type] } {
      VMOV 3 mom_kin_spindle_axis mom_sys_spindle_axis
   } elseif { ![string compare "Table" $mom_kin_5th_axis_type] } {
      VMOV 3 axis vec
      if { ![string compare "XY" $mom_kin_4th_axis_plane] } {
         set vec(2) 0.0
      } elseif { ![string compare "ZX" $mom_kin_4th_axis_plane] } {
         set vec(1) 0.0
      } elseif { ![string compare "YZ" $mom_kin_4th_axis_plane] } {
         set vec(0) 0.0
      }
      set len [VEC3_unitize vec mom_sys_spindle_axis]
      if { [EQ_is_zero $len] } { set mom_sys_spindle_axis(2) 1.0 }
   } else {
      VMOV 3 axis mom_sys_spindle_axis
   }
}


#=============================================================
proc GET_SUBOP_MOVE_NAME { move_type } {
#=============================================================
# Aug-15-2018 gsl - Return GMC subop name of interOp path
#
   switch $move_type {
      10 {
         set move_name "Tool_Change_Container"
      }
      11 {
         set move_name "Tool_Change_Position"
      }
      12 {
         set move_name "Rotary_Tool_Center_Point_On"
      }
      13 {
         set move_name "Rotary_Tool_Center_Point_Off"
      }
      115 {
         set move_name "Move_to_Machine_Position"
      }
      810 {
         set move_name "Rotary_Point_Vector_Move"
      }
      default {
         set move_name "Unknown"
      }
   }

return $move_name
}


#=============================================================
proc HANDLE_FIRST_LINEAR_MOVE { } {
#=============================================================
# Called by MOM_linear_move to handle the 1st linear move of an operation.
#
   if { ![info exists ::first_linear_move] } {
      set ::first_linear_move 0
   }
   if { !$::first_linear_move } {
      PB_first_linear_move
      incr ::first_linear_move
   }
}


#=============================================================
proc INFO { args } {
#=============================================================
   MOM_output_to_listing_device [join $args]
}


#=============================================================================
proc INIT_VAR { VAR {_val 0} } {
#=============================================================================
# 10/31/2018 gsl - Initialize a variable if absent.
#
# - By default, the variable in question will be initialized to "0",
#   unless a desired value is supplied.
#
  upvar $VAR _var
   if { ![info exists _var] } { set _var $_val }
}


#=============================================================
proc LIMIT_ANGLE { a } {
#=============================================================

   set a [expr fmod($a,360)]
   set a [expr ($a < 0) ? ($a + 360) : $a]

return $a
}


#=============================================================
proc LINEARIZE_LOCK_MOTION { } {
#=============================================================
# called by LOCK_AXIS_MOTION
#
#  This command linearizes the move between two positions that
#  have both linear and rotary motion.  The rotary motion is
#  created by LOCK_AXIS from the coordinates in the locked plane.
#  The combined linear and rotary moves result in non-linear
#  motion.  This command will break the move into shorter moves
#  that do not violate the tolerance.
#
#<04-08-2014 gsl> - Corrected error with use of mom_outangle_pos.
#<12-03-2014 gsl> - Declaration of global unlocked_pos & unlocked_prev_pos were commented out in pb903.
#<09-09-2015 ljt> - Ensure mom_prev_pos is locked, and raise warning
#                   when linearization iteration does not complete.
# Sep-11-2017 gsl - 7698782 Undo revision of <09-09-2015 ljt>. Ref revision in LOCK_AXIS_MOTION.

   global mom_pos
   global mom_prev_pos
   global unlocked_pos
   global unlocked_prev_pos
   global mom_kin_linearization_tol
   global mom_kin_machine_resolution
   global mom_out_angle_pos

   VMOV 5 mom_pos locked_pos

   # <09-Sep-2015 ljt> Make sure mom_prev_pos is locked. If mom_pos has been reloaded and
   #                   when MOM_POST_convert_point is called in core result can be wrong.
   #<Sep-11-2017 gsl> 7698782
    VMOV 5 mom_prev_pos locked_prev_pos
   # LOCK_AXIS mom_prev_pos locked_prev_pos ::mom_prev_out_angle_pos

   UNLOCK_AXIS locked_pos unlocked_pos
   UNLOCK_AXIS locked_prev_pos unlocked_prev_pos

   VMOV 5 unlocked_pos save_unlocked_pos
   VMOV 5 locked_pos save_locked_pos

   set loop 0
   set count 0

   set tol $mom_kin_linearization_tol

   while { $loop == 0 } {

      for { set i 3 } { $i < 5 } { incr i } {
         set del [expr $locked_pos($i) - $locked_prev_pos($i)]
         if { $del > 180.0 } {
            set locked_prev_pos($i) [expr $locked_prev_pos($i) + 360.0]
         } elseif { $del < -180.0 } {
            set locked_prev_pos($i) [expr $locked_prev_pos($i) - 360.0]
         }
      }

      set loop 1

      for { set i 0 } { $i < 5 } { incr i } {
         set mid_unlocked_pos($i) [expr ( $unlocked_pos($i) + $unlocked_prev_pos($i) )/2.0]
         set mid_locked_pos($i) [expr ( $locked_pos($i) + $locked_prev_pos($i) )/2.0]
      }

      UNLOCK_AXIS mid_locked_pos temp

      VEC3_sub temp mid_unlocked_pos work

      set error [VEC3_mag work]

      if { $count > 20 } {

         VMOV 5 locked_pos mom_pos
         VMOV 5 unlocked_pos mom_prev_pos

         CATCH_WARNING "LINEARIZATION ITERATION FAILED."

         LINEARIZE_LOCK_OUTPUT $count

      } elseif { $error < $tol } {

         VMOV 5 locked_pos mom_pos
         VMOV 5 unlocked_pos mom_prev_pos

         CATCH_WARNING "LINEARIZATION ITERATION FAILED."

         LINEARIZE_LOCK_OUTPUT $count

         VMOV 5 unlocked_pos unlocked_prev_pos
         VMOV 5 locked_pos locked_prev_pos

         if { $count != 0 } {
            VMOV 5 save_unlocked_pos unlocked_pos
            VMOV 5 save_locked_pos locked_pos
            set loop 0
            set count 0
         }

      } else {

         if { $error < $mom_kin_machine_resolution } {
            set error $mom_kin_machine_resolution
         }

         set error [expr sqrt( $tol*.98/$error )]

         if { $error < .5 } { set error .5 }

         for { set i 0 } { $i < 5 } { incr i } {
            set locked_pos($i)   [expr $locked_prev_pos($i)   + ( $locked_pos($i)   - $locked_prev_pos($i)   )*$error]
            set unlocked_pos($i) [expr $unlocked_prev_pos($i) + ( $unlocked_pos($i) - $unlocked_prev_pos($i) )*$error]
         }

        #<04-08-2014 gsl> mom_out_angle_pos was mom_outangle_pos.
         LOCK_AXIS unlocked_pos locked_pos mom_out_angle_pos

         set loop 0
         incr count
      }
   }

#<04-08-2014 gsl> Didn't make difference
#   MOM_reload_variable -a mom_pos
#   MOM_reload_variable -a mom_prev_pos
#   MOM_reload_variable -a mom_out_angle_pos
}


#=============================================================
proc LINEARIZE_LOCK_OUTPUT { count } {
#=============================================================
# called by LOCK_AXIS_MOTION & LINEARIZE_LOCK_MOTION
# "count > 0" will cause output.
#
# Jul-16-2013     - pb1003
# Oct-15-2015 ljt - PR6789060, account for reversed rotation, reload mom_prev_rot_ang_4/5th
# Oct-26-2018 gsl - 8961308 : Recompute DPM
#
   global mom_pos mom_prev_pos
   global mom_mcs_goto mom_prev_mcs_goto
   global mom_out_angle_pos mom_prev_out_angle_pos
   global mom_prev_rot_ang_4th
   global mom_prev_rot_ang_5th
   global mom_kin_4th_axis_direction
   global mom_kin_5th_axis_direction
   global mom_kin_4th_axis_leader
   global mom_kin_5th_axis_leader
   global mom_sys_leader
   global mom_motion_distance
   global mom_feed_rate_number
   global mom_feed_rate
   global mom_kin_machine_resolution
   global mom_kin_max_frn
   global mom_kin_machine_type
   global mom_kin_4th_axis_min_limit mom_kin_4th_axis_max_limit
   global mom_kin_5th_axis_min_limit mom_kin_5th_axis_max_limit
   global unlocked_pos unlocked_prev_pos


   set mom_out_angle_pos(0)  [ROTSET $mom_pos(3) $mom_prev_rot_ang_4th $mom_kin_4th_axis_direction\
                                     $mom_kin_4th_axis_leader mom_sys_leader(fourth_axis)\
                                     $mom_kin_4th_axis_min_limit $mom_kin_4th_axis_max_limit]

  # Make sure previous angles are correct which will be used in next ROTSET.
   set mom_prev_rot_ang_4th $mom_out_angle_pos(0)
   MOM_reload_variable mom_prev_rot_ang_4th

   if { [string match "5_axis_*table" $mom_kin_machine_type] } {

     # Account for reversed rotation, mom_kin_5th_axis_vector is always the positive direction of x/y/z,
     # only fifth axis can be locked for five axis post, and the tool axis is parallel to mom_kin_5th_axis_vector
     # if the tool axis leads to the negative direction, the angle need to be reversed.
      if { [string match "MAGNITUDE_DETERMINES_DIRECTION" $mom_kin_5th_axis_direction]\
           && [VEC3_dot ::mom_tool_axis ::mom_kin_5th_axis_vector] < 0 } {

         set mom_pos(4) [expr -1 * $mom_pos(4)]
      }
      set mom_out_angle_pos(1)  [ROTSET $mom_pos(4) $mom_prev_rot_ang_5th $mom_kin_5th_axis_direction\
                                        $mom_kin_5th_axis_leader mom_sys_leader(fifth_axis)\
                                        $mom_kin_5th_axis_min_limit $mom_kin_5th_axis_max_limit]

      set mom_prev_rot_ang_5th $mom_out_angle_pos(1)
      MOM_reload_variable mom_prev_rot_ang_5th
   }

  #
  #  Re-calcualte the distance and feed rate number
  #
   if { $count < 0 } {
      VEC3_sub mom_mcs_goto mom_prev_mcs_goto delta
   } else {
      VEC3_sub unlocked_pos unlocked_prev_pos delta
   }

   set mom_motion_distance [VEC3_mag delta]

  # Raw FRN w/o constraint of max DPM
   if { [EQ_is_lt $mom_motion_distance $mom_kin_machine_resolution] } {
      set mom_feed_rate_number $mom_kin_max_frn
   } else {
      set mom_feed_rate_number [expr $mom_feed_rate / $mom_motion_distance]
   }

   set mom_pos(3) $mom_out_angle_pos(0)
   set mom_pos(4) $mom_out_angle_pos(1)

   MOM_reload_variable -a mom_out_angle_pos


  #<10/26/2018 gsl> Recompute DPM during LOCK_AXIS
   global mom_feed_rate_dpm
   global mom_sys_rotary_axis_index

   set prev_ang [LIMIT_ANGLE $::mom_prev_out_angle_pos([expr $mom_sys_rotary_axis_index - 3 ])]
   set curr_ang [LIMIT_ANGLE $::mom_out_angle_pos([expr $mom_sys_rotary_axis_index - 3 ])]
   if { $prev_ang > 180.0 } { set prev_ang [LIMIT_ANGLE [expr -$prev_ang]] }
   if { $curr_ang > 180.0 } { set curr_ang [LIMIT_ANGLE [expr -$curr_ang]] }
   set minrot   [expr abs( $curr_ang - $prev_ang )]

   set mom_feed_rate_dpm [expr $minrot * $mom_feed_rate_number]
   if { [expr $mom_feed_rate_dpm > $::mom_kin_max_dpm ] } {
      set mom_feed_rate_dpm $::mom_kin_max_dpm
   }
   MOM_reload_variable mom_feed_rate_dpm


  # Normalized for feedrate mode determination
   set mom_pos($mom_sys_rotary_axis_index)      [LIMIT_ANGLE $mom_pos($mom_sys_rotary_axis_index)]
   set mom_prev_pos($mom_sys_rotary_axis_index) [LIMIT_ANGLE $mom_prev_pos($mom_sys_rotary_axis_index)]


   FEEDRATE_SET

   if { $count > 0 } { PB_CMD_linear_move }
}


#=============================================================
proc LOCK_AXIS { input_point output_point output_rotary } {
#=============================================================
# called by LOCK_AXIS_MOTION & LINEARIZE_LOCK_MOTION
#
# (pb903)
# 09-06-13 Allen - PR6932644 - implement lock axis for 4 axis machine.
# 04-16-14 gsl   - Account for offsets resulted from right-angled head attachment
# 09-09-15 ljt   - Replace mom_kin_4/5th_axis_center_offset with mom_kin_4/5th_axis_point
# 10-15-15 ljt   - PR6789060, account for reversed rotation of table not perpendicular to spindle axis.

   upvar $input_point in_pos ; upvar $output_point out_pos ; upvar $output_rotary or

   global mom_kin_4th_axis_center_offset
   global mom_kin_5th_axis_center_offset
   global mom_sys_lock_value
   global mom_sys_lock_plane
   global mom_sys_lock_axis
   global mom_sys_unlocked_axis
   global mom_sys_4th_axis_index
   global mom_sys_5th_axis_index
   global mom_sys_linear_axis_index_1
   global mom_sys_linear_axis_index_2
   global mom_sys_rotary_axis_index
   global mom_kin_machine_resolution
   global mom_prev_lock_angle
   global mom_out_angle_pos
   global mom_prev_rot_ang_4th
   global mom_prev_rot_ang_5th
   global positive_radius
   global DEG2RAD
   global RAD2DEG
   global mom_kin_4th_axis_rotation
   global mom_kin_5th_axis_rotation
   global mom_kin_machine_type
   global mom_kin_4th_axis_point
   global mom_kin_5th_axis_point
   global mom_origin


   if { ![info exists positive_radius] } { set positive_radius 0 }

   if { $mom_sys_rotary_axis_index == 3 } {
      if { ![info exists mom_prev_rot_ang_4th] } { set mom_prev_rot_ang_4th 0.0 }
      set mom_prev_lock_angle $mom_prev_rot_ang_4th
   } else {
      if { ![info exists mom_prev_rot_ang_5th] } { set mom_prev_rot_ang_5th 0.0 }
      set mom_prev_lock_angle $mom_prev_rot_ang_5th
   }

  #<04-16-2014 gsl> Add offsets of angled-head attachment to input point
   VMOV 5 in_pos ip
   ACCOUNT_HEAD_OFFSETS ip 1


   # <09-Sep-2015 ljt> Add offsets of 4/5th axis rotary center
   VMOV 3 ip temp
   if { [CMD_EXIST MOM_validate_machine_model] \
        && [string match "TRUE" [MOM_validate_machine_model]] } {

      if { [string match "5_axis_*table" $mom_kin_machine_type] && [info exists mom_kin_5th_axis_point] } {

         VEC3_sub temp mom_kin_5th_axis_point temp

      } elseif { ( [string match "4_axis_table" $mom_kin_machine_type] || [string match "*mill_turn" $mom_kin_machine_type] ) \
                 && [info exists mom_kin_4th_axis_point] } {

         VEC3_sub temp mom_kin_4th_axis_point temp
      }

   } else {

      # mom_origin is a vector from table center to destination MCS
      if { [info exists mom_origin] } {
         VEC3_add temp mom_origin temp
      }

      if { [info exists mom_kin_4th_axis_center_offset] } {
         VEC3_sub temp mom_kin_4th_axis_center_offset temp
      }

      if { [info exists mom_kin_5th_axis_center_offset ] } {
         VEC3_sub temp mom_kin_5th_axis_center_offset temp
      }
   }

   set temp(3) $ip(3)
   set temp(4) $ip(4)

   if { $mom_sys_lock_axis > 2 } {

      set angle [expr ($mom_sys_lock_value - $temp($mom_sys_lock_axis))*$DEG2RAD]
      ROTATE_VECTOR $mom_sys_lock_plane $angle temp temp1
      VMOV 3 temp1 temp
      set temp($mom_sys_lock_axis) $mom_sys_lock_value

   } else {

      # <15-Oct-15 ljt> lock plane is 5th axis plane for 5axis machine
      if { [string match "5_axis_*table" $mom_kin_machine_type] } {
         set angle [expr ($temp(4))*$DEG2RAD]

         # <03-11-10 wbh> 6308668 Check the rotation mode
         if [string match "reverse" $mom_kin_5th_axis_rotation] {
            set angle [expr -$angle]
         }

         ROTATE_VECTOR $mom_sys_5th_axis_index $angle temp temp1
         VMOV 3 temp1 temp
         set temp(4) 0.0
      }


      #<09-06-13 Allen> Fix PR6932644 to implement lock axis for 4 axis machine.
      #<11-15-2013 gsl> ==> Rotation seemed to be reversed!
      if { [string match "4_axis_*" $mom_kin_machine_type] } {
         if { ![string compare $mom_sys_lock_plane $mom_sys_4th_axis_index] } {
            set angle [expr $temp(3)*$DEG2RAD]
            if [string match "reverse" $mom_kin_4th_axis_rotation] {
               set angle [expr -$angle]
            }

            ROTATE_VECTOR $mom_sys_4th_axis_index $angle temp temp1

            VMOV 3 temp1 temp
            set temp(3) 0.0
         }
      }


      set rad [expr sqrt($temp($mom_sys_linear_axis_index_1)*$temp($mom_sys_linear_axis_index_1) +\
                         $temp($mom_sys_linear_axis_index_2)*$temp($mom_sys_linear_axis_index_2))]

      set angle [ARCTAN $temp($mom_sys_linear_axis_index_2) $temp($mom_sys_linear_axis_index_1)]

      # <03-11-10 wbh> 6308668 Check the rotation mode
      # <15-Oct-15 ljt> lock plane is 5th axis plane for 5axis machine
      if { [string match "5_axis_*table" $mom_kin_machine_type] } {
         if [string match "reverse" $mom_kin_5th_axis_rotation] {
            set angle [expr -$angle]
         }
      } elseif { ![string compare $mom_sys_lock_plane $mom_sys_4th_axis_index] } {
         if [string match "reverse" $mom_kin_4th_axis_rotation] {
            set angle [expr -$angle]
         }
      }

      if { $rad < [expr abs($mom_sys_lock_value) + $mom_kin_machine_resolution] } {
         if { $mom_sys_lock_value < 0.0 } {
            set temp($mom_sys_lock_axis) [expr -$rad]
         } else {
            set temp($mom_sys_lock_axis) $rad
         }
      } else {
         set temp($mom_sys_lock_axis) $mom_sys_lock_value
      }

      set temp($mom_sys_unlocked_axis)  [expr sqrt($rad*$rad - $temp($mom_sys_lock_axis)*$temp($mom_sys_lock_axis))]

      VMOV 5 temp temp1
      set temp1($mom_sys_unlocked_axis) [expr -$temp($mom_sys_unlocked_axis)]
      set ang1 [ARCTAN $temp($mom_sys_linear_axis_index_2)  $temp($mom_sys_linear_axis_index_1)]
      set ang2 [ARCTAN $temp1($mom_sys_linear_axis_index_2) $temp1($mom_sys_linear_axis_index_1)]
      set temp($mom_sys_rotary_axis_index)  [expr ($angle - $ang1)*$RAD2DEG]
      set temp1($mom_sys_rotary_axis_index) [expr ($angle - $ang2)*$RAD2DEG]
      set ang1 [LIMIT_ANGLE [expr $mom_prev_lock_angle - $temp($mom_sys_rotary_axis_index)]]
      set ang2 [LIMIT_ANGLE [expr $mom_prev_lock_angle - $temp1($mom_sys_rotary_axis_index)]]

      if { $ang1 > 180.0 } { set ang1 [LIMIT_ANGLE [expr -$ang1]] }
      if { $ang2 > 180.0 } { set ang2 [LIMIT_ANGLE [expr -$ang2]] }

      if { $positive_radius == 0 } {
         if { $ang1 > $ang2 } {
            VMOV 5 temp1 temp
            set positive_radius "-1"
         } else {
            set positive_radius "1"
         }
      } elseif { $positive_radius == -1 } {
         VMOV 5 temp1 temp
      }

     #+++++++++++++++++++++++++++++++++++++++++
     # NOT needed!!! <= will cause misbehavior
     # VMOV 5 temp1 temp
   }


   # <09-Sep-2015 ljt> Remove offsets of the 4/5th axis rotary center
   VMOV 3 temp op
   if { [CMD_EXIST MOM_validate_machine_model] \
        && [string match "TRUE" [MOM_validate_machine_model]] } {

      if { [string match "5_axis_*table" $mom_kin_machine_type] && [info exists mom_kin_5th_axis_point] } {

         VEC3_add op mom_kin_5th_axis_point op

      } elseif { ( [string match "4_axis_table" $mom_kin_machine_type] || [string match "*mill_turn" $mom_kin_machine_type] ) \
                 && [info exists mom_kin_4th_axis_point] } {

         VEC3_add op mom_kin_4th_axis_point op
      }

   } else {

      if { [info exists mom_origin] } {
         VEC3_sub op mom_origin op
      }

      if { [info exists mom_kin_4th_axis_center_offset] } {
         VEC3_add op mom_kin_4th_axis_center_offset op
      }

      if { [info exists mom_kin_5th_axis_center_offset] } {
         VEC3_add op mom_kin_5th_axis_center_offset op
      }

   }

   if { ![info exists or] } {
      set or(0) 0.0
      set or(1) 0.0
   }

   set mom_prev_lock_angle $temp($mom_sys_rotary_axis_index)
   set op(3) $temp(3)
   set op(4) $temp(4)


  #<04-16-2014 gsl> Remove offsets of angled-head attachment from output point
   ACCOUNT_HEAD_OFFSETS op 0
   VMOV 5 op out_pos
}


#=============================================================
proc LOCK_AXIS_INITIALIZE { } {
#=============================================================
# called by MOM_lock_axis
# ==> It's only used by MOM_lock_axis, perhaps it should be defined within.

   global mom_sys_lock_plane
   global mom_sys_lock_axis
   global mom_sys_unlocked_axis
   global mom_sys_unlock_plane
   global mom_sys_4th_axis_index
   global mom_sys_5th_axis_index
   global mom_sys_linear_axis_index_1
   global mom_sys_linear_axis_index_2
   global mom_sys_rotary_axis_index
   global mom_kin_4th_axis_plane
   global mom_kin_5th_axis_plane
   global mom_kin_machine_type

   if { $mom_sys_lock_plane == -1 } {
      if { ![string compare "XY" $mom_kin_4th_axis_plane] } {
         set mom_sys_lock_plane 2
      } elseif { ![string compare "ZX" $mom_kin_4th_axis_plane] } {
         set mom_sys_lock_plane 1
      } elseif { ![string compare "YZ" $mom_kin_4th_axis_plane] } {
         set mom_sys_lock_plane 0
      }
   }

   set mom_sys_4th_axis_index -1
   if { ![string compare "XY" $mom_kin_4th_axis_plane] } {
      set mom_sys_4th_axis_index 2
   } elseif { ![string compare "ZX" $mom_kin_4th_axis_plane] } {
      set mom_sys_4th_axis_index 1
   } elseif { ![string compare "YZ" $mom_kin_4th_axis_plane] } {
      set mom_sys_4th_axis_index 0
   }


  # Check whether the machine type is 5-axis.
   set mom_sys_5th_axis_index -1
   if { [string match "5_axis_*" $mom_kin_machine_type] && [info exists mom_kin_5th_axis_plane] } {
      if { ![string compare "XY" $mom_kin_5th_axis_plane] } {
         set mom_sys_5th_axis_index 2
      } elseif { ![string compare "ZX" $mom_kin_5th_axis_plane] } {
         set mom_sys_5th_axis_index 1
      } elseif { ![string compare "YZ" $mom_kin_5th_axis_plane] } {
         set mom_sys_5th_axis_index 0
      }
   }


   if { $mom_sys_lock_plane == 0 } {
      set mom_sys_linear_axis_index_1 1
      set mom_sys_linear_axis_index_2 2
   } elseif { $mom_sys_lock_plane == 1 } {
      set mom_sys_linear_axis_index_1 2
      set mom_sys_linear_axis_index_2 0
   } elseif { $mom_sys_lock_plane == 2 } {
      set mom_sys_linear_axis_index_1 0
      set mom_sys_linear_axis_index_2 1
   }

   # Can only lock the last rotary axis
   if { $mom_sys_5th_axis_index == -1 } {
      set mom_sys_rotary_axis_index 3
   } else {
      set mom_sys_rotary_axis_index 4
   }

   set mom_sys_unlocked_axis [expr $mom_sys_linear_axis_index_1 +\
                                   $mom_sys_linear_axis_index_2 -\
                                   $mom_sys_lock_axis]


#MOM_output_text "( >>> mom_sys_lock_plane          : $mom_sys_lock_plane )"
#MOM_output_text "( >>> mom_sys_lock_axis           : $mom_sys_lock_axis )"
#MOM_output_text "( >>> mom_sys_unlocked_axis       : $mom_sys_unlocked_axis )"
#MOM_output_text "( >>> mom_sys_4th_axis_index      : $mom_sys_4th_axis_index )"
#MOM_output_text "( >>> mom_sys_5th_axis_index      : $mom_sys_5th_axis_index )"
#MOM_output_text "( >>> mom_sys_linear_axis_index_1 : $mom_sys_linear_axis_index_1 )"
#MOM_output_text "( >>> mom_sys_linear_axis_index_2 : $mom_sys_linear_axis_index_2 )"
#MOM_output_text "( >>> mom_sys_rotary_axis_index   : $mom_sys_rotary_axis_index )"
#MOM_output_text "( >>> mom_kin_4th_axis_plane      : $mom_kin_4th_axis_plane )"
#MOM_output_text "( >>> mom_kin_5th_axis_plane      : $mom_kin_5th_axis_plane )"
#MOM_output_text "( >>> mom_kin_machine_type        : $mom_kin_machine_type )"
}


#=============================================================
proc LOCK_AXIS_MOTION { } {
#=============================================================
# called by PB_CMD_kin_before_motion
#
#  The UDE lock_axis must be specified in the tool path
#  for the post to lock the requested axis.  The UDE lock_axis may only
#  be used for four and five axis machine tools.  A four axis post may
#  only lock an axis in the plane of the fourth axis.  For five axis
#  posts, only the fifth axis may be locked.  Five axis will only
#  output correctly if the fifth axis is rotated so it is perpendicular
#  to the spindle axis.
#
# Mar-29-2016     - Of NX/PB v11.0
# Sep-11-2017 gsl - 7698782 Also recompute mom_prev_pos with LOCK_AXIS

  # Must be called by PB_CMD_kin_before_motion
   if { ![CALLED_BY "PB_CMD_kin_before_motion"] } {
return
   }


   if { [string match "circular_move" $::mom_current_motion] } {
return
   }



   global mom_sys_lock_status

   if { [string match "ON" $mom_sys_lock_status] } {

      global mom_pos mom_out_angle_pos
      global mom_motion_type
      global mom_cycle_feed_to_pos
      global mom_cycle_feed_to mom_tool_axis
      global mom_motion_event
      global mom_cycle_rapid_to_pos
      global mom_cycle_retract_to_pos
      global mom_cycle_rapid_to
      global mom_cycle_retract_to
      global mom_prev_pos
      global mom_kin_4th_axis_type
      global mom_kin_spindle_axis
      global mom_kin_5th_axis_type
      global mom_kin_4th_axis_plane
      global mom_sys_cycle_after_initial
      global mom_kin_4th_axis_min_limit
      global mom_kin_4th_axis_max_limit
      global mom_kin_5th_axis_min_limit
      global mom_kin_5th_axis_max_limit
      global mom_prev_rot_ang_4th
      global mom_prev_rot_ang_5th
      global mom_kin_4th_axis_direction
      global mom_kin_5th_axis_direction
      global mom_kin_4th_axis_leader
      global mom_kin_5th_axis_leader
      global mom_kin_machine_type


      if { ![info exists mom_sys_cycle_after_initial] } {
         set mom_sys_cycle_after_initial "FALSE"
      }

      if { [string match "FALSE" $mom_sys_cycle_after_initial] } {
         LOCK_AXIS mom_pos mom_pos mom_out_angle_pos

        #<Sep-11-2017 gsl> 7698782 Also recompute mom_prev_pos
         LOCK_AXIS mom_prev_pos mom_prev_pos ::mom_prev_out_angle_pos
      }

      if { [string match "CYCLE" $mom_motion_type] } {

         if { [string match "Table" $mom_kin_4th_axis_type] } {

           # "mom_spindle_axis" would have the head attachment incorporated.
            global mom_spindle_axis
            if [info exists mom_spindle_axis] {
               VMOV 3 mom_spindle_axis mom_sys_spindle_axis
            } else {
               VMOV 3 mom_kin_spindle_axis mom_sys_spindle_axis
            }

         } elseif { [string match "Table" $mom_kin_5th_axis_type] } {

            VMOV 3 mom_tool_axis vec

           # Zero component of rotating axis
            switch $mom_kin_4th_axis_plane {
               XY {
                  set vec(2) 0.0
               }
               ZX {
                  set vec(1) 0.0
               }
               YZ {
                  set vec(0) 0.0
               }
            }

           # Reworked logic to prevent potential error
            set len [VEC3_mag vec]
            if { [EQ_is_gt $len 0.0] } {
               VEC3_unitize vec mom_sys_spindle_axis
            } else {
               set mom_sys_spindle_axis(0) 0.0
               set mom_sys_spindle_axis(1) 0.0
               set mom_sys_spindle_axis(2) 1.0
            }

         } else {

            VMOV 3 mom_tool_axis mom_sys_spindle_axis
         }

         set mom_cycle_feed_to_pos(0)    [expr $mom_pos(0) + $mom_cycle_feed_to    * $mom_sys_spindle_axis(0)]
         set mom_cycle_feed_to_pos(1)    [expr $mom_pos(1) + $mom_cycle_feed_to    * $mom_sys_spindle_axis(1)]
         set mom_cycle_feed_to_pos(2)    [expr $mom_pos(2) + $mom_cycle_feed_to    * $mom_sys_spindle_axis(2)]

         set mom_cycle_rapid_to_pos(0)   [expr $mom_pos(0) + $mom_cycle_rapid_to   * $mom_sys_spindle_axis(0)]
         set mom_cycle_rapid_to_pos(1)   [expr $mom_pos(1) + $mom_cycle_rapid_to   * $mom_sys_spindle_axis(1)]
         set mom_cycle_rapid_to_pos(2)   [expr $mom_pos(2) + $mom_cycle_rapid_to   * $mom_sys_spindle_axis(2)]

         set mom_cycle_retract_to_pos(0) [expr $mom_pos(0) + $mom_cycle_retract_to * $mom_sys_spindle_axis(0)]
         set mom_cycle_retract_to_pos(1) [expr $mom_pos(1) + $mom_cycle_retract_to * $mom_sys_spindle_axis(1)]
         set mom_cycle_retract_to_pos(2) [expr $mom_pos(2) + $mom_cycle_retract_to * $mom_sys_spindle_axis(2)]
      }


      global mom_kin_linearization_flag

      if { ![string compare "TRUE"       $mom_kin_linearization_flag] &&\
            [string compare "RAPID"      $mom_motion_type]            &&\
            [string compare "CYCLE"      $mom_motion_type]            &&\
            [string compare "rapid_move" $mom_motion_event] } {

         LINEARIZE_LOCK_MOTION

      } else {

         if { ![info exists mom_prev_rot_ang_4th] } { set mom_prev_rot_ang_4th 0.0 }
         if { ![info exists mom_prev_rot_ang_5th] } { set mom_prev_rot_ang_5th 0.0 }

         LINEARIZE_LOCK_OUTPUT -1
      }


     #VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
     # > Do not reload mom_pos here!
     # MOM_reload_variable -a mom_pos

   } ;# mom_sys_lock_status
}


#=============================================================
proc LOCK_AXIS_SUB { axis } {
#=============================================================
# called by SET_LOCK

  global mom_pos mom_lock_axis_value_defined mom_lock_axis_value

   if { $mom_lock_axis_value_defined == 1 } {
      return $mom_lock_axis_value
   } else {
      return $mom_pos($axis)
   }
}


#=============================================================
proc LOCK_AXIS__pb901 { input_point output_point output_rotary } {
#=============================================================
# called by LOCK_AXIS_MOTION & LINEARIZE_LOCK_MOTION

   upvar $input_point ip ; upvar $output_point op ; upvar $output_rotary or

   global mom_kin_machine_type
   global mom_kin_4th_axis_center_offset
   global mom_kin_5th_axis_center_offset
   global mom_sys_lock_value
   global mom_sys_lock_plane
   global mom_sys_lock_axis
   global mom_sys_unlocked_axis
   global mom_sys_4th_axis_index
   global mom_sys_5th_axis_index
   global mom_sys_linear_axis_index_1
   global mom_sys_linear_axis_index_2
   global mom_sys_rotary_axis_index
   global mom_kin_machine_resolution
   global mom_prev_lock_angle
   global mom_out_angle_pos
   global mom_prev_rot_ang_4th
   global mom_prev_rot_ang_5th
   global positive_radius
   global DEG2RAD
   global RAD2DEG
   global mom_kin_4th_axis_rotation
   global mom_kin_5th_axis_rotation

   if { ![info exists positive_radius] } { set positive_radius 0 }

   if { $mom_sys_rotary_axis_index == 3 } {
      if { ![info exists mom_prev_rot_ang_4th] } { set mom_prev_rot_ang_4th 0.0 }
      set mom_prev_lock_angle $mom_prev_rot_ang_4th
   } else {
      if { ![info exists mom_prev_rot_ang_5th] } { set mom_prev_rot_ang_5th 0.0 }
      set mom_prev_lock_angle $mom_prev_rot_ang_5th
   }

   if { ![info exists mom_kin_4th_axis_center_offset] } {
      set temp(0) $ip(0)
      set temp(1) $ip(1)
      set temp(2) $ip(2)
   } else {
      VEC3_sub ip mom_kin_4th_axis_center_offset temp
   }

   if { [info exists mom_kin_5th_axis_center_offset] } {
      VEC3_sub temp mom_kin_5th_axis_center_offset temp
   }

   set temp(3) $ip(3)
   set temp(4) $ip(4)

   if { $mom_sys_lock_axis > 2 } {
      set angle [expr ( $mom_sys_lock_value - $temp($mom_sys_lock_axis) )*$DEG2RAD]
      ROTATE_VECTOR $mom_sys_lock_plane $angle temp temp1
      VMOV 3 temp1 temp
      set temp($mom_sys_lock_axis) $mom_sys_lock_value
   } else {
      if { ![string compare $mom_sys_lock_plane $mom_sys_5th_axis_index] } {
         set angle [expr $temp(4)*$DEG2RAD]

         # <03-11-10 wbh> 6308668 Check the rotation mode
         if [string match "reverse" $mom_kin_5th_axis_rotation] {
            set angle [expr -$angle]
         }

         ROTATE_VECTOR $mom_sys_5th_axis_index $angle temp temp1
         VMOV 3 temp1 temp
         set temp(4) 0.0
      }

      #<09-06-13 Allen> Lock axis for 4-axis machine.
      if { [string match "4_axis_*" $mom_kin_machine_type] } {
         if { ![string compare $mom_sys_lock_plane $mom_sys_4th_axis_index] } {
            set angle [expr $temp(3) * $DEG2RAD]
            if [string match "reverse" $mom_kin_4th_axis_rotation] {
               set angle [expr -$angle]
            }

            ROTATE_VECTOR $mom_sys_4th_axis_index $angle temp temp1
            VMOV 3 temp1 temp
            set temp(3) 0.0
         }
      }

      set rad [expr sqrt( $temp($mom_sys_linear_axis_index_1) * $temp($mom_sys_linear_axis_index_1) +\
                          $temp($mom_sys_linear_axis_index_2) * $temp($mom_sys_linear_axis_index_2) )]
      set angle [ARCTAN $temp($mom_sys_linear_axis_index_2) $temp($mom_sys_linear_axis_index_1)]

      # <03-11-10 wbh> 6308668 Check the rotation mode
      if { ![string compare $mom_sys_lock_plane $mom_sys_5th_axis_index] } {
         if [string match "reverse" $mom_kin_5th_axis_rotation] {
            set angle [expr -$angle]
         }
      } elseif { ![string compare $mom_sys_lock_plane $mom_sys_4th_axis_index] } {
         if [string match "reverse" $mom_kin_4th_axis_rotation] {
            set angle [expr -$angle]
         }
      }

      if { $rad < [expr abs($mom_sys_lock_value) + $mom_kin_machine_resolution] } {
         if { $mom_sys_lock_value < 0.0 } {
            set temp($mom_sys_lock_axis) [expr -$rad]
         } else {
            set temp($mom_sys_lock_axis) $rad
         }
      } else {
         set temp($mom_sys_lock_axis) $mom_sys_lock_value
      }

      set temp($mom_sys_unlocked_axis)  [expr sqrt($rad*$rad - $temp($mom_sys_lock_axis)*$temp($mom_sys_lock_axis))]

      VMOV 5 temp temp1
      set temp1($mom_sys_unlocked_axis) [expr -$temp($mom_sys_unlocked_axis)]

      set ang1 [ARCTAN $temp($mom_sys_linear_axis_index_2) $temp($mom_sys_linear_axis_index_1)]
      set ang2 [ARCTAN $temp1($mom_sys_linear_axis_index_2) $temp1($mom_sys_linear_axis_index_1)]

      set temp($mom_sys_rotary_axis_index)  [expr ($angle-$ang1)*$RAD2DEG]
      set temp1($mom_sys_rotary_axis_index) [expr ($angle-$ang2)*$RAD2DEG]

      set ang1 [LIMIT_ANGLE [expr $mom_prev_lock_angle - $temp($mom_sys_rotary_axis_index)]]
      set ang2 [LIMIT_ANGLE [expr $mom_prev_lock_angle - $temp1($mom_sys_rotary_axis_index)]]

      if { $ang1 > 180.0 } { set ang1 [LIMIT_ANGLE [expr -$ang1]] }
      if { $ang2 > 180.0 } { set ang2 [LIMIT_ANGLE [expr -$ang2]] }

      if { $positive_radius == 0 } {
         if { $ang1 > $ang2 } {
            VMOV 5 temp1 temp
            set positive_radius "-1"
         } else {
            set positive_radius "1"
         }
      } elseif { $positive_radius == -1 } {
         VMOV 5 temp1 temp
      }
   }

   if { [info exists mom_kin_4th_axis_center_offset] } {
      VEC3_add temp mom_kin_4th_axis_center_offset op
   } else {
      set op(0) $temp(0)
      set op(1) $temp(1)
      set op(2) $temp(2)
   }

   if { [info exists mom_kin_5th_axis_center_offset] } {
      VEC3_add op mom_kin_5th_axis_center_offset op
   }

   if { ![info exists or] } {
      set or(0) 0.0
      set or(1) 0.0
   }

   set mom_prev_lock_angle $temp($mom_sys_rotary_axis_index)
   set op(3) $temp(3)
   set op(4) $temp(4)
}


#=============================================================
proc MAXMIN_ANGLE { a max min {tol_flag 0} } {
#=============================================================

   if { $tol_flag == 0 } { ;# Direct comparison

      while { $a < $min } { set a [expr $a + 360.0] }
      while { $a > $max } { set a [expr $a - 360.0] }

   } else { ;# Tolerant comparison

      while { [EQ_is_lt $a $min] } { set a [expr $a + 360.0] }
      while { [EQ_is_gt $a $max] } { set a [expr $a - 360.0] }
   }

return $a
}


#-------------------------------------------------------------
proc MTX2_det { MTX2 } {
#-------------------------------------------------------------
   upvar $MTX2  m2

return [expr $m2(0)*$m2(3) - $m2(1)*$m2(2)]
}


#-------------------------------------------------------------
proc MTX3_det { MTX3 } {
#-------------------------------------------------------------
   upvar $MTX3  m3

return [expr $m3(0)*( $m3(4)*$m3(8) - $m3(5)*$m3(7) ) - \
             $m3(3)*( $m3(1)*$m3(8) - $m3(2)*$m3(7) ) + \
             $m3(6)*( $m3(1)*$m3(5) - $m3(2)*$m3(4) )]
}


#-------------------------------------------------------------
proc MTX3_init { CSYS } {
#-------------------------------------------------------------
   upvar $CSYS  csys

   set csys(0) 1.0;  set csys(1) 0.0;  set csys(2) 0.0
   set csys(3) 0.0;  set csys(4) 1.0;  set csys(5) 0.0
   set csys(6) 0.0;  set csys(7) 0.0;  set csys(8) 1.0
}


#-------------------------------------------------------------
proc MTX3_invert { CSYS ISYS } {
#-------------------------------------------------------------
   upvar $CSYS  csys
   upvar $ISYS  isys

   set a11 $csys(0);  set a12 $csys(1);  set a13 $csys(2)
   set a21 $csys(3);  set a22 $csys(4);  set a23 $csys(5)
   set a31 $csys(6);  set a32 $csys(7);  set a33 $csys(8)

   set m3_det [MTX3_det csys]

   for { set i 0 } { $i < 9 } { incr i } {
      switch $i {
         0 { set m2(0) $a22; set m2(1) $a23; set m2(2) $a32; set m2(3) $a33 }
         1 { set m2(0) $a13; set m2(1) $a12; set m2(2) $a33; set m2(3) $a32 }
         2 { set m2(0) $a12; set m2(1) $a13; set m2(2) $a22; set m2(3) $a23 }
         3 { set m2(0) $a23; set m2(1) $a21; set m2(2) $a33; set m2(3) $a31 }
         4 { set m2(0) $a11; set m2(1) $a13; set m2(2) $a31; set m2(3) $a33 }
         5 { set m2(0) $a13; set m2(1) $a11; set m2(2) $a23; set m2(3) $a21 }
         6 { set m2(0) $a21; set m2(1) $a22; set m2(2) $a31; set m2(3) $a32 }
         7 { set m2(0) $a12; set m2(1) $a11; set m2(2) $a32; set m2(3) $a31 }
         8 { set m2(0) $a11; set m2(1) $a12; set m2(2) $a21; set m2(3) $a22 }
         default {}
      }

      set isys($i) [expr [MTX2_det m2] / $m3_det]
   }
}


#=============================================================
proc MTX3_xform_csys { a b c x y z CSYS } {
#=============================================================
   upvar $CSYS csys
#
#                     Rotation about
#         X                  Y                  Z
# ----------------   ----------------   ----------------
#   1     0     0     cosB   0   sinB    cosC -sinC    0
#   0   cosA -sinA     0     1     0     sinC  cosC    0
#   0   sinA  cosA   -sinB   0   cosB      0     0     1

   if { [EQ_is_zero $a] && [EQ_is_zero $b] && [EQ_is_zero $c] && [EQ_is_zero $x] && [EQ_is_zero $y] && [EQ_is_zero $z] } {
return
   }


   set xa [expr -1*$a]
   set yb [expr -1*$b]
   set zc [expr -1*$c]

  #<12-14-05 gsl>
   for { set i 0 } { $i < 9 } { incr i } {
      set mm($i) $csys($i)
   }

  # Rotate about X
   set mx(0) 1;           set mx(1) 0;           set mx(2) 0
   set mx(3) 0;           set mx(4) [cosD $xa];  set mx(5) [-sinD $xa]
   set mx(6) 0;           set mx(7) [sinD $xa];  set mx(8) [cosD $xa]

  # Rotate about Y
   set my(0) [cosD $yb];  set my(1) 0;           set my(2) [sinD $yb]
   set my(3) 0;           set my(4) 1;           set my(5) 0
   set my(6) [-sinD $yb]; set my(7) 0;           set my(8) [cosD $yb]

  # Rotate about Z
   set mz(0) [cosD $zc];  set mz(1) [-sinD $zc]; set mz(2) 0;
   set mz(3) [sinD $zc];  set mz(4) [cosD $zc];  set mz(5) 0;
   set mz(6) 0;           set mz(7) 0;           set mz(8) 1

#   MTX3_multiply  mz my ma
#   MTX3_multiply  ma mx mt
#   MTX3_multiply  mt mm m2
#   MTX3_transpose m2 csys

   MTX3_multiply  my mx ma
   MTX3_transpose ma mt
   MTX3_multiply  mz mt ma
   MTX3_multiply  ma mm m2
   MTX3_transpose m2 csys

  #<12-14-05 gsl> Add translation
   set csys(9)  [expr $csys(9)  + $x]
   set csys(10) [expr $csys(10) + $y]
   set csys(11) [expr $csys(11) + $z]
}


#=============================================================
proc MTX4_multiply { m n r } {
#=============================================================
  #r = ( m X n )      Matrix multiplication, m, n, r are 4X4 metrix
  upvar $m m1 ; upvar $n n1 ; upvar $r r1
  set r1(0) [expr ($m1(0) * $n1(0) + $m1(4) * $n1(1) + $m1(8) * $n1(2) + $m1(12) * $n1(3))]
  set r1(1) [expr ($m1(1) * $n1(0) + $m1(5) * $n1(1) + $m1(9) * $n1(2) + $m1(13) * $n1(3))]
  set r1(2) [expr ($m1(2) * $n1(0) + $m1(6) * $n1(1) + $m1(10)* $n1(2) + $m1(14) * $n1(3))]
  set r1(3) [expr ($m1(3) * $n1(0) + $m1(7) * $n1(1) + $m1(11)* $n1(2) + $m1(15) * $n1(3))]

  set r1(4) [expr ($m1(0) * $n1(4) + $m1(4) * $n1(5) + $m1(8) * $n1(6) + $m1(12) * $n1(7))]
  set r1(5) [expr ($m1(1) * $n1(4) + $m1(5) * $n1(5) + $m1(9) * $n1(6) + $m1(13) * $n1(7))]
  set r1(6) [expr ($m1(2) * $n1(4) + $m1(6) * $n1(5) + $m1(10)* $n1(6) + $m1(14) * $n1(7))]
  set r1(7) [expr ($m1(3) * $n1(4) + $m1(7) * $n1(5) + $m1(11)* $n1(6) + $m1(15) * $n1(7))]

  set r1(8) [expr ($m1(0) * $n1(8) + $m1(4) * $n1(9) + $m1(8) * $n1(10) + $m1(12) * $n1(11))]
  set r1(9) [expr ($m1(1) * $n1(8) + $m1(5) * $n1(9) + $m1(9) * $n1(10) + $m1(13) * $n1(11))]
  set r1(10) [expr ($m1(2) * $n1(8) + $m1(6) * $n1(9) + $m1(10)* $n1(10) + $m1(14) * $n1(11))]
  set r1(11) [expr ($m1(3) * $n1(8) + $m1(7) * $n1(9) + $m1(11)* $n1(10) + $m1(15) * $n1(11))]

  set r1(12) [expr ($m1(0) * $n1(12) + $m1(4) * $n1(13) + $m1(8) * $n1(14) + $m1(12) * $n1(15))]
  set r1(13) [expr ($m1(1) * $n1(12) + $m1(5) * $n1(13) + $m1(9) * $n1(14) + $m1(13) * $n1(15))]
  set r1(14) [expr ($m1(2) * $n1(12) + $m1(6) * $n1(13) + $m1(10)* $n1(14) + $m1(14) * $n1(15))]
  set r1(15) [expr ($m1(3) * $n1(12) + $m1(7) * $n1(13) + $m1(11)* $n1(14) + $m1(15) * $n1(15))]
}


#=============================================================
proc OPERATOR_MSG { msg {seq_no 0} } {
#=============================================================
# This command will output a single or a set of operator message(s).
#
#   msg    : Message(s separated by new-line character)
#   seq_no : 0 Output message without sequence number (Default)
#            1 Output message with sequence number
#

   foreach s [split $msg \n] {
      set s1 "$::mom_sys_control_out $s $::mom_sys_control_in"
      if { !$seq_no } {
         MOM_suppress once N
      }
      MOM_output_literal $s1
   }

   set ::mom_o_buffer ""
}


#=============================================================
proc OPERATOR_MSG_debug { msg {seq_no 0} } {
#=============================================================
# This command will output a single or a set of operator message(s).
#
#   msg    : Message(s separated by new-line character)
#   seq_no : 0 Output message without sequence number (Default)
#            1 Output message with sequence number
#

# - UnComment next line to suppress debug messages upon release of this post -
 return

   OPERATOR_MSG $msg $seq_no
}


#=============================================================
proc OPL { args } {
#=============================================================
   MOM_output_literal [join $args]
}


#=============================================================
proc OUTPUT_MACRO { macro_string suppress_seqno } {
#=============================================================
# This command will be called in PB_call_macro to split a macro string
# into multiple lines of output by the separator "{n}" token.
#
# 03-04-2019 gsl - New
#

# - Uncomment next line to display debug message -
# OPERATOR_MSG "MACRO_STRING : $macro_string"

   set seqno_status [MOM_set_seq_off]
   if { [string match "on" $seqno_status] } { MOM_set_seq_on }

   if { [string match "on" $seqno_status] && $suppress_seqno } {
      set suppress_seqno 1
   } else {
      set suppress_seqno 0
   }


  # Define split_str -
   set split_str "\{n\}"

   set string_list [split $macro_string ${split_str}]
   set list_len [llength $string_list]

   set blank ""

   set i 0
   foreach s $string_list {
      if { $i == 0 } {
         if { $suppress_seqno } {
            MOM_suppress once N
            MOM_output_literal $s
         } else {
            MOM_output_literal $s

            if { $list_len > 1 } {
               set n_blank [expr [string length [MOM_ask_address_value N]] +\
                                 [string length $::mom_sys_word_separator] +\
                                 [string length $::mom_sys_leader(N)]]

               if { $n_blank > 0 } {
                  append blank [format %${n_blank}c 32]
               }
            }
         }
      } else {
         if [string match "on" $seqno_status] {
            MOM_suppress once N
         }
         if { $suppress_seqno } {
            MOM_output_literal $s
         } else {
            MOM_output_literal "${blank}$s"
         }
      }
      incr i
   }
}


#=============================================================
proc PAUSE { args } {
#=============================================================
# Revisions:
#-----------
# 05-19-10 gsl - Use EXEC command
#

   global env

   if { [info exists env(PB_SUPPRESS_UGPOST_DEBUG)]  &&  $env(PB_SUPPRESS_UGPOST_DEBUG) == 1 } {
  return
   }


   global gPB

   if { [info exists gPB(PB_disable_MOM_pause)]  &&  $gPB(PB_disable_MOM_pause) == 1 } {
  return
   }


   global tcl_platform

   set cam_aux_dir  [MOM_ask_env_var UGII_CAM_AUXILIARY_DIR]

   if { [string match "*windows*" $tcl_platform(platform)] } {
      set ug_wish "ugwish.exe"
   } else {
      set ug_wish ugwish
   }

   if { [file exists ${cam_aux_dir}$ug_wish]  &&  [file exists ${cam_aux_dir}mom_pause.tcl] } {

      set title ""
      set msg ""

      if { [llength $args] == 1 } {
         set msg [lindex $args 0]
      }

      if { [llength $args] > 1 } {
         set title [lindex $args 0]
         set msg [lindex $args 1]
      }

      set command_string [concat \"${cam_aux_dir}$ug_wish\" \"${cam_aux_dir}mom_pause.tcl\" \"$title\" \"$msg\"]

      set res [EXEC $command_string]


      switch [string trim $res] {
         no {
            set gPB(PB_disable_MOM_pause) 1
         }
         cancel {
            set gPB(PB_disable_MOM_pause) 1

            uplevel #0 {
               if { [CMD_EXIST MOM_abort_program] } {
                  MOM_abort_program "*** User Abort Post Processing *** "
               } else {
                  MOM_abort "*** User Abort Post Processing *** "
               }
            }
         }
         default {
            return
         }
      }

   } else {

      CATCH_WARNING "PAUSE not executed -- \"$ug_wish\" or \"mom_pause.tcl\" not found"
   }
}


#=============================================================
proc PAUSE_win64 { args } {
#=============================================================
   global env
   if { [info exists env(PB_SUPPRESS_UGPOST_DEBUG)]  &&  $env(PB_SUPPRESS_UGPOST_DEBUG) == 1 } {
  return
   }

   global gPB
   if { [info exists gPB(PB_disable_MOM_pause)]  &&  $gPB(PB_disable_MOM_pause) == 1 } {
  return
   }


   set cam_aux_dir  [MOM_ask_env_var UGII_CAM_AUXILIARY_DIR]
   set ug_wish "ugwish.exe"

   if { [file exists ${cam_aux_dir}$ug_wish] &&\
        [file exists ${cam_aux_dir}mom_pause_win64.tcl] } {

      set title ""
      set msg ""

      if { [llength $args] == 1 } {
         set msg [lindex $args 0]
      }

      if { [llength $args] > 1 } {
         set title [lindex $args 0]
         set msg [lindex $args 1]
      }


     ######
     # Define a scratch file and pass it to mom_pause_win64.tcl script -
     #
     #   A separated process will be created to construct the Tk dialog.
     #   This process will communicate with the main process via the state of a scratch file.
     #   This scratch file will collect the messages that need to be conveyed from the
     #   child process to the main process.
     ######
      global mom_logname
      set pause_file_name "$env(TEMP)/${mom_logname}_mom_pause_[clock clicks].txt"


     ######
     # Path names should be per unix style for "open" command
     ######
      regsub -all {\\} $pause_file_name {/}  pause_file_name
      regsub -all { }  $pause_file_name {\ } pause_file_name
      regsub -all {\\} $cam_aux_dir {/}  cam_aux_dir
      regsub -all { }  $cam_aux_dir {\ } cam_aux_dir

      if [file exists $pause_file_name] {
         file delete -force $pause_file_name
      }


     ######
     # Note that the argument order for mom_pasue.tcl has been changed
     # The assumption at this point is we will always have the communication file as the first
     # argument and optionally the title and message as the second and third arguments
     ######
      open "|${cam_aux_dir}$ug_wish ${cam_aux_dir}mom_pause_win64.tcl ${pause_file_name} {$title} {$msg}"


     ######
     # Waiting for the mom_pause to complete its process...
     # - This is indicated when the scratch file materialized and became read-only.
     ######
      while { ![file exists $pause_file_name] || [file writable $pause_file_name] } { }


     ######
     # Delay a 100 milli-seconds to ensure that sufficient time is given for the other process to complete.
     ######
      after 100


     ######
     # Open the scratch file to read and process the information.  Close it afterward.
     ######
      set fid [open "$pause_file_name" r]

      set res [string trim [gets $fid]]
      switch $res {
         no {
            set gPB(PB_disable_MOM_pause) 1
         }
         cancel {
            close $fid
            file delete -force $pause_file_name

            set gPB(PB_disable_MOM_pause) 1

            uplevel #0 {
               if { [CMD_EXIST MOM_abort_program] } {
                  MOM_abort_program "*** User Abort Post Processing *** "
               } else {
                  MOM_abort "*** User Abort Post Processing *** "
               }
            }
         }
         default {}
      }


     ######
     # Delete the scratch file
     ######
      close $fid
      file delete -force $pause_file_name
   }
}


#=============================================================
proc PAUSE_x { args } {
#=============================================================
   global env
   if { [info exists env(PB_SUPPRESS_UGPOST_DEBUG)]  &&  $env(PB_SUPPRESS_UGPOST_DEBUG) == 1 } {
  return
   }

   global gPB
   if { [info exists gPB(PB_disable_MOM_pause)]  &&  $gPB(PB_disable_MOM_pause) == 1 } {
  return
   }



  #==========
  # Win64 OS
  #
   global tcl_platform

   if { [string match "*windows*" $tcl_platform(platform)] } {
      global mom_sys_processor_archit

      if { ![info exists mom_sys_processor_archit] } {
         set pVal ""
         set env_vars [array get env]
         set idx [lsearch $env_vars "PROCESSOR_ARCHITE*"]
         if { $idx >= 0 } {
            set pVar [lindex $env_vars $idx]
            set pVal [lindex $env_vars [expr $idx + 1]]
         }
         set mom_sys_processor_archit $pVal
      }

      if { [string match "*64*" $mom_sys_processor_archit] } {

         PAUSE_win64 $args
  return
      }
   }



   set cam_aux_dir  [MOM_ask_env_var UGII_CAM_AUXILIARY_DIR]


   if { [string match "*windows*" $tcl_platform(platform)] } {
     set ug_wish "ugwish.exe"
   } else {
     set ug_wish ugwish
   }

   if { [file exists ${cam_aux_dir}$ug_wish] && [file exists ${cam_aux_dir}mom_pause.tcl] } {

      set title ""
      set msg ""

      if { [llength $args] == 1 } {
         set msg [lindex $args 0]
      }

      if { [llength $args] > 1 } {
         set title [lindex $args 0]
         set msg [lindex $args 1]
      }

      set res [exec ${cam_aux_dir}$ug_wish ${cam_aux_dir}mom_pause.tcl $title $msg]
      switch $res {
         no {
            set gPB(PB_disable_MOM_pause) 1
         }
         cancel {
            set gPB(PB_disable_MOM_pause) 1

            uplevel #0 {
               MOM_abort "*** User Abort Post Processing *** "
            }
         }
         default { return }
      }
   }
}


#=============================================================
proc PREFERRED_SOLUTION { } {
#=============================================================
# To be called by PB_CMD_kin_before_motion
# ==> Perhaps, after the 4-axis output validation!
# ==> Not yet released officially
#
#  UDE "Set Preferred Solution" can be specified with the operation in question.
#  This event will be handled before "Lock Axis" to choose, possibly,
#  the alternate solution of a 5-axis motion based on the perferred
#  delimiter (mom_preferred_zone_flag) such as X/Y-plus(or minus) or
#  4th/5th-angle etc. Choices can be
#
#    [XPLUS | XMINUS | YPLUS | YMINUS | FOURTH | FIFTH].
#
#
#  => Should this flag be in effect forever until cancelled by
#     another instance of the same UDE that turns it off?
#  => Initial rotary angle can be influenced by using a "Rotate" UDE.
#
#
   if [CMD_EXIST PB_CMD__choose_preferred_solution] {
      PB_CMD__choose_preferred_solution
   }
}


#=============================================================
proc REPOSITION_ERROR_CHECK { warn } {
#=============================================================
# not called in this script

   global mom_kin_4th_axis_max_limit mom_kin_4th_axis_min_limit
   global mom_kin_5th_axis_max_limit mom_kin_5th_axis_min_limit
   global mom_pos mom_prev_pos mom_alt_pos mom_alt_prev_pos
   global mom_sys_rotary_error mom_warning_info mom_kin_machine_type

   if { [string compare "secondary rotary position being used" $warn] || [string index $mom_kin_machine_type 0] != 5 } {
      set mom_sys_rotary_error $warn
return
   }

   set mom_sys_rotary_error 0

   set a4 [expr $mom_alt_pos(3)+360.0]
   set a5 [expr $mom_alt_pos(4)+360.0]

   while { [expr $a4-$mom_kin_4th_axis_min_limit] > 360.0 } { set a4 [expr $a4-360.0] }
   while { [expr $a5-$mom_kin_5th_axis_min_limit] > 360.0 } { set a5 [expr $a5-360.0] }

   if { $a4 <= $mom_kin_4th_axis_max_limit && $a5 <= $mom_kin_5th_axis_max_limit } {
return
   }

   for { set i 0 } { $i < 2 } { incr i } {
      set rot($i) [expr $mom_pos([expr $i+3]) - $mom_prev_pos([expr $i+3])]
      while { $rot($i) > 180.0 } { set rot($i) [expr $rot($i)-360.0] }
      while { $rot($i) < 180.0 } { set rot($i) [expr $rot($i)+360.0] }
      set rot($i) [expr abs($rot($i))]

      set rotalt($i) [expr $mom_alt_pos([expr $i+3]) - $mom_prev_pos([expr $i+3])]
      while { $rotalt($i) > 180.0 } { set rotalt($i) [expr $rotalt($i)-360.0] }
      while { $rotalt($i) < 180.0 } { set rotalt($i) [expr $rotalt($i)+360.0] }
      set rotalt($i) [expr abs($rotalt($i))]
   }

   if { [EQ_is_equal [expr $rot(0)+$rot(1)] [expr $rotalt(0)+$rotalt(1)]] } {
return
   }

   set mom_sys_rotary_error $warn
}


#=============================================================
proc RESET_DPP_VALUE { } {
#=============================================================
# Set default dpp values, these values will be changed by system. User shouldn't do customization here.
#
# 05-14-2013 levi - Initial version

  global dpp_ge

## dpp_ge(toolpath_axis_num)
## "3"   3 axis tool path
## "5"   5 axis tool path

## dpp_ge(coord_rot)
## "LOCAL"        the programming coordinate system is CSYS
## "AUTO_3D"      the programming coordinate system is determined by tool axis which is not parallel to Z axis
## "NONE"         it's not a 3+2 axis machining operation

## dpp_ge(prev_coord_offset)
## the linear offset of last G68

## dpp_ge(prev_coord_rot_angle)
## an array records the coordinate rotation angles by previous swiveling function

## dpp_ge(prev_g68_first_vec)
## the vector stored the last G68 first vector

## dpp_ge(prev_g68_second_vec)
## the vector stored the last G68 second vector

## dpp_ge(cycle_hole_counter)
## this variable is used to record hole counter

  set dpp_ge(toolpath_axis_num) "3"
  set dpp_ge(coord_rot) "NONE"
  set dpp_ge(prev_coord_offset,0) 0
  set dpp_ge(prev_coord_offset,1) 0
  set dpp_ge(prev_coord_offset,2) 0
  set dpp_ge(prev_coord_rot_angle,0) 0
  set dpp_ge(prev_coord_rot_angle,1) 0
  set dpp_ge(prev_coord_rot_angle,2) 0
  set dpp_ge(prev_g68_first_vec,0) 0
  set dpp_ge(prev_g68_first_vec,1) 0
  set dpp_ge(prev_g68_first_vec,2) 0
  set dpp_ge(prev_g68_second_vec,0) 0
  set dpp_ge(prev_g68_second_vec,1) 0
  set dpp_ge(prev_g68_second_vec,2) 0
  set dpp_ge(cycle_hole_counter) 0
}


#=============================================================
proc RESET_ROTARY_SIGN { ang pre_ang axis } {
#=============================================================
# Called by ROTARY_AXIS_RETRACT
#
# The input parameters "ang" & "pre_ang" must use same unit. (Both in degree or radian)

   global mom_kin_4th_axis_direction mom_kin_5th_axis_direction
   global mom_kin_4th_axis_rotation mom_kin_5th_axis_rotation
   global mom_rotary_direction_4th mom_rotary_direction_5th

   set abs_ang [expr abs($ang)]
   set abs_pre [expr abs($pre_ang)]
   if { $axis == 3 && ![string compare "SIGN_DETERMINES_DIRECTION" $mom_kin_4th_axis_direction] } {
    # The fourth axis.
      if { $abs_ang > $abs_pre } {
         set mom_rotary_direction_4th 1
      } elseif { $abs_ang < $abs_pre } {
         set mom_rotary_direction_4th -1
      }
   } elseif { $axis == 4 && ![string compare "SIGN_DETERMINES_DIRECTION" $mom_kin_5th_axis_direction] } {
    # The fifth axis.
      if { $abs_ang > $abs_pre } {
         set mom_rotary_direction_5th 1
      } elseif { $abs_ang < $abs_pre } {
         set mom_rotary_direction_5th -1
      }
   }
}


#=============================================================
proc RETRACT_POINT_CHECK { refpt axis retpt } {
#=============================================================
# called by CALC_SPHERICAL_RETRACT_POINT & CALC_CYLINDRICAL_RETRACT_POINT

  upvar $refpt rfp ; upvar $axis ax ; upvar $retpt rtp

#
#  determine if retraction point is "below" the retraction plane
#  if the tool is already in a safe position, do not retract
#
#  return 0    no retract needed
#         1     retraction needed
#

   VEC3_sub rtp rfp vec
   if { [VEC3_is_zero vec] } {
return 0
   }

   set x [VEC3_unitize vec vec1]
   set dir [VEC3_dot ax vec1]

   if { $dir <= 0.0 } {
return 0
   } else {
return 1
   }
}


#=============================================================
proc ROTARY_AXIS_RETRACT { } {
#=============================================================
# called by PB_CMD_kin_before_motion
#
#  This command is used by four and five axis posts to retract
#  from workpiece when the rotary axis become discontinuous.
#  This command is activated by setting the axis limit violation
#  action to "Retract / Re-Engage".
#
#-------------------------------------------------------------
# Nov-30-2016 gsl - (pb11.02) Corrected logic
# Sep-11-2017 gsl - (pb12.01) PB_user_defined_axis_limit_action was PB_user_def_axis_limit_action.
# Apr-13-2018 gsl - (pb12.02) Enhanced check condition for axis_limit_action.
# Aug-14-2019 gsl - (pb1899)  Patch up potentially missing ::mom_kin_5th_axis_limit_action
#

  #(pb903) Removed restriction below; command may be used in other situations
  # Must be called by PB_CMD_kin_before_motion
  if 0 {
   if { ![CALLED_BY "PB_CMD_kin_before_motion"] } {
 return
   }
  }

   global mom_sys_rotary_error
   global mom_motion_event


   if { ![info exists mom_sys_rotary_error] } {
return
   }

   set rotary_error_code $mom_sys_rotary_error

  # Make sure mom_sys_rotary_error is always unset.
   unset mom_sys_rotary_error


   if { [info exists mom_motion_event] } {

      if { $rotary_error_code != 0 && ![string compare "linear_move" $mom_motion_event] } {

        #<06-25-12 gsl> The above conditions have been checked in PB_CMD_kin_before_motion already.

         global mom_kin_reengage_distance
         global mom_kin_rotary_reengage_feedrate
         global mom_kin_rapid_feed_rate
         global mom_pos
         global mom_prev_pos
         global mom_prev_rot_ang_4th mom_prev_rot_ang_5th
         global mom_kin_4th_axis_direction mom_kin_4th_axis_leader
         global mom_out_angle_pos mom_kin_5th_axis_direction mom_kin_5th_axis_leader
         global mom_kin_4th_axis_center_offset mom_kin_5th_axis_center_offset
         global mom_sys_leader mom_tool_axis mom_prev_tool_axis mom_kin_4th_axis_type
         global mom_kin_spindle_axis
         global mom_alt_pos mom_prev_alt_pos mom_feed_rate
         global mom_kin_rotary_reengage_feedrate
         global mom_feed_engage_value mom_feed_cut_value
         global mom_warning_info
         global mom_kin_4th_axis_min_limit mom_kin_4th_axis_max_limit
         global mom_kin_5th_axis_min_limit mom_kin_5th_axis_max_limit

         global mom_kin_machine_type


        # (pb1899) - Patch up potentially missing 5th axis variables used in this command
         if { ![string match "5_axis_*" $mom_kin_machine_type] } {
            if { ![info exists ::mom_kin_5th_axis_limit_action] } {
               set ::mom_kin_5th_axis_limit_action $::mom_kin_4th_axis_limit_action
            }
         }


        #
        #  Check for the limit action being warning only.  If so, issue warning and leave
        #
        # (pb12.02) - Enhanced check condition
         if { ![string compare "Warning" $::mom_kin_4th_axis_limit_action] &&\
              ![string compare "Warning" $::mom_kin_5th_axis_limit_action] } {

            CATCH_WARNING "Rotary axis limit violated, discontinuous motion may result."

            return

         } elseif { ![string compare "User Defined" $::mom_kin_4th_axis_limit_action] ||\
                    ![string compare "User Defined" $::mom_kin_5th_axis_limit_action] } {

            PB_user_defined_axis_limit_action

            return
         }

        #
        #  The previous rotary info is only available after the first motion.
        #
         if { ![info exists mom_prev_rot_ang_4th] } {
            set mom_prev_rot_ang_4th [MOM_ask_address_value fourth_axis]
         }

         if { ![info exists mom_prev_rot_ang_5th] } {
            set mom_prev_rot_ang_5th [MOM_ask_address_value fifth_axis]
         }

        #
        #  Determine the type of rotary violation encountered.  There are
        #  three distinct possibilities.
        #
        #  "ROTARY CROSSING LIMIT" with a four axis machine tool.  The fourth
        #      axis will be repositioned by either +360 or -360 before
        #      re-engaging (roterr = 0).
        #
        #  "ROTARY CROSSING LIMIT" with a five axis machine tool.  There are two
        #      possible solutions.  If the axis that crossed a limit can be
        #      repositioned by adding or subtracting 360, then that solution
        #      will be used (roterr = 0).  If there is only one position available and it is
        #      not in the valid travel limits, then the alternate position will
        #      be tested.  If valid, then the "secondary rotary position being used"
        #      method will be used (roterr = 2).
        #      If the alternate position cannot be used, a warning will be given.
        #
        #  "secondary rotary position being used" can only occur with a five
        #      axis machine tool.  The tool will reposition to the alternate
        #      current rotary position and re-engage to the alternate current
        #      linear position (roterr = 1).
        #
        #
        #    roterr = 0 :
        #      Rotary Reposition : mom_prev_pos(3,4) +- 360
        #      Linear Re-Engage :  mom_prev_pos(0,1,2)
        #      Final End Point :   mom_pos(0-4)
        #
        #    roterr = 1 :
        #      Rotary Reposition : mom_prev_alt_pos(3,4)
        #      Linear Re-Engage :  mom_prev_alt_pos(0,1,2)
        #      Final End Point :   mom_pos(0-4)
        #
        #    roterr = 2 :
        #      Rotary Reposition : mom_prev_alt_pos(3,4)
        #      Linear Re-Engage :  mom_prev_alt_pos(0,1,2)
        #      Final End Point :   mom_alt_pos(0-4)
        #
        #    For all cases, a warning will be given if it is not possible to
        #    to cut from the re-calculated previous position to move end point.
        #    For all valid cases the tool will, retract from the part, reposition
        #    the rotary axis and re-engage back to the part.
        #

         if { ![string compare "ROTARY CROSSING LIMIT." $rotary_error_code] } {

            if { [string match "5_axis_*" [string tolower $mom_kin_machine_type]] } {

               set d [expr $mom_out_angle_pos(0) - $mom_prev_rot_ang_4th]

               if { [expr abs($d)] > 180.0 } {
                  set min $mom_kin_4th_axis_min_limit
                  set max $mom_kin_4th_axis_max_limit
                  if { $d > 0.0 } {
                     set ang [expr $mom_prev_rot_ang_4th + 360.0]
                  } else {
                     set ang [expr $mom_prev_rot_ang_4th - 360.0]
                  }
               } else {
                  set min $mom_kin_5th_axis_min_limit
                  set max $mom_kin_5th_axis_max_limit
                  set d [expr $mom_out_angle_pos(1) - $mom_prev_rot_ang_5th]
                  if { $d > 0.0 } {
                     set ang [expr $mom_prev_rot_ang_5th + 360.0]
                  } else {
                     set ang [expr $mom_prev_rot_ang_5th - 360.0]
                  }
               }

               if { $ang >= $min && $ang <= $max } { ;# ==> 5th axis min/max will be used here(?)
                  set roterr 0
               } else {
                  set roterr 2
               }
            } else {
               set roterr 0
            }

         } else {

            set roterr 1
         }

        #
        #  Retract from part
        #
         VMOV 5 mom_pos save_pos
         VMOV 5 mom_prev_pos save_prev_pos
         VMOV 2 mom_out_angle_pos save_out_angle_pos
         set save_feedrate $mom_feed_rate

         global mom_kin_output_unit mom_part_unit
         if { ![string compare $mom_kin_output_unit $mom_part_unit] } {
            set mom_sys_unit_conversion "1.0"
         } elseif { ![string compare "IN" $mom_kin_output_unit] } {
            set mom_sys_unit_conversion [expr 1.0/25.4]
         } else {
            set mom_sys_unit_conversion 25.4
         }

        #<01-07-10 wbh> Fix pr6192146.
        # Declare/Set the variables used to convert the feed rate from MMPR/IPR to MMPM/IPM.
         global mom_spindle_rpm
         global mom_feed_approach_unit mom_feed_cut_unit
         global mom_feed_engage_unit mom_feed_retract_unit

         set mode_convert_scale "1.0"
         if { [info exists mom_spindle_rpm] && [EQ_is_gt $mom_spindle_rpm 0.0] } {
            set mode_convert_scale $mom_spindle_rpm
         }

         global mom_sys_spindle_axis
         GET_SPINDLE_AXIS mom_prev_tool_axis

         global mom_kin_retract_type
         global mom_kin_retract_distance
         global mom_kin_retract_plane

         if { ![info exists mom_kin_retract_distance] } {
            if { [info exists mom_kin_retract_plane] } {
              # Convert legacy variable
               set mom_kin_retract_distance $mom_kin_retract_plane
            } else {
               set mom_kin_retract_distance 10.0
            }
         }

         if { ![info exists mom_kin_retract_type] } {
            set mom_kin_retract_type "DISTANCE"
         }

        #<Nov-30-2016 gsl> (pb1102) Enforce retract type for machines only with table(s).
         set machine_type [string tolower $::mom_kin_machine_type]
         switch $machine_type {
            4_axis_table -
            5_axis_dual_table {
               set mom_kin_retract_type "DISTANCE"
            }
         }

        #
        #  Pre-release type conversion
        #
         if { [string match "PLANE" $mom_kin_retract_type] } {
            set mom_kin_retract_type "SURFACE"
         }

         switch $mom_kin_retract_type {
            SURFACE {
               set cen(0) 0.0
               set cen(1) 0.0
               set cen(2) 0.0

               if { [info exists mom_kin_4th_axis_center_offset] } {
                  VEC3_add cen mom_kin_4th_axis_center_offset cen
               }

              #<Nov-30-2016 gsl> (pb1102) Is logic below proper?
              if 0 {
               if { ![string compare "Table" $mom_kin_4th_axis_type] } {
                  set num_sol [CALC_CYLINDRICAL_RETRACT_POINT mom_prev_pos mom_kin_spindle_axis\
                                                              $mom_kin_retract_distance ret_pt]
               } else {
                  set num_sol [CALC_SPHERICAL_RETRACT_POINT   mom_prev_pos mom_prev_tool_axis cen\
                                                              $mom_kin_retract_distance ret_pt]
               }
              }

               set machine_type [string tolower $::mom_kin_machine_type]
               switch $machine_type {
                  4_axis_head -
                  5_axis_head_table {
                     set num_sol [CALC_CYLINDRICAL_RETRACT_POINT mom_prev_pos mom_kin_spindle_axis\
                                                                 $mom_kin_retract_distance ret_pt]
                  }
                  5_axis_dual_head {
                     set num_sol [CALC_SPHERICAL_RETRACT_POINT   mom_prev_pos mom_prev_tool_axis cen\
                                                                 $mom_kin_retract_distance ret_pt]
                  }
               }

               if { $num_sol != 0 } { VEC3_add ret_pt cen mom_pos }
            }

            DISTANCE -
            default {
               set mom_pos(0) [expr $mom_prev_pos(0) + $mom_kin_retract_distance*$mom_sys_spindle_axis(0)]
               set mom_pos(1) [expr $mom_prev_pos(1) + $mom_kin_retract_distance*$mom_sys_spindle_axis(1)]
               set mom_pos(2) [expr $mom_prev_pos(2) + $mom_kin_retract_distance*$mom_sys_spindle_axis(2)]
               set num_sol 1
            }
         }


         global mom_motion_distance
         global mom_feed_rate_number
         global mom_feed_retract_value
         global mom_feed_approach_value


         set dist [expr $mom_kin_reengage_distance*2.0]

         if { $num_sol != 0 } {
        #
        #  Retract from the part at rapid feed rate.  This is the same for all conditions.
        #
            MOM_suppress once fourth_axis fifth_axis
            set mom_feed_rate [expr $mom_feed_retract_value*$mom_sys_unit_conversion]

           #<01-07-10 wbh> Convert the feed rate from MMPR/IPR to MMPM/IPM
            if { [info exists mom_feed_retract_unit] && [string match "*pr" $mom_feed_retract_unit] } {
               set mom_feed_rate [expr $mom_feed_rate * $mode_convert_scale]
            }
            if { [EQ_is_equal $mom_feed_rate 0.0] } {
               set mom_feed_rate [expr $mom_kin_rapid_feed_rate*$mom_sys_unit_conversion]
            }

            VEC3_sub mom_pos mom_prev_pos del_pos
            set dist [VEC3_mag del_pos]

           #<03-13-08 gsl> Replaced next call
           # global mom_sys_frn_factor
           # set mom_feed_rate_number [expr ($mom_sys_frn_factor*$mom_feed_rate)/ $dist]
            set mom_feed_rate_number [SET_FEEDRATE_NUMBER $dist $mom_feed_rate]
            FEEDRATE_SET
            set retract "yes"

         } else {

            CATCH_WARNING "Retraction geometry is defined inside of the current point.\n\
                           No retraction will be output. Set the retraction distance to a greater value."
            set retract "no"
         }

         if { $roterr == 0 } {
#
#  This section of code handles the case where a limit forces a reposition to an angle
#  by adding or subtracting 360 until the new angle is within the limits.
#  This is either a four axis case or a five axis case where it is not a problem
#  with the inverse kinematics forcing a change of solution.
#  This is only a case of "unwinding" the table.
#
            if { ![string compare "yes"  $retract] } {
               PB_CMD_retract_move
            }

           #
           #  Move to previous rotary position
           #  <04-01-2013 gsl> mom_rev_pos(3,4) may have not been affected, we may just borrow them
           #                   as mom_out_angle_pos for subsequent output instead of recomputing them thru ROTSET(?)
           #
            if { [info exists mom_kin_4th_axis_direction] } {
               set mom_out_angle_pos(0) [ROTSET $mom_prev_pos(3) $mom_out_angle_pos(0) $mom_kin_4th_axis_direction\
                                                $mom_kin_4th_axis_leader mom_sys_leader(fourth_axis)\
                                                $mom_kin_4th_axis_min_limit $mom_kin_4th_axis_max_limit]
            }
            if { [info exists mom_kin_5th_axis_direction] } {
               set mom_out_angle_pos(1) [ROTSET $mom_prev_pos(4) $mom_out_angle_pos(1) $mom_kin_5th_axis_direction\
                                                $mom_kin_5th_axis_leader mom_sys_leader(fifth_axis)\
                                                $mom_kin_5th_axis_min_limit $mom_kin_5th_axis_max_limit]
            }

            PB_CMD_reposition_move

           #
           #  Position back to part at approach feed rate
           #
            GET_SPINDLE_AXIS mom_prev_tool_axis
            for { set i 0 } { $i < 3 } { incr i } {
               set mom_pos($i) [expr $mom_prev_pos($i) + $mom_kin_reengage_distance * $mom_sys_spindle_axis($i)]
            }
            set mom_feed_rate [expr $mom_feed_approach_value * $mom_sys_unit_conversion]
           #<01-07-10 wbh> Convert the feed rate from MMPR/IPR to MMPM/IPM
            if { [info exists mom_feed_approach_unit] && [string match "*pr" $mom_feed_approach_unit] } {
               set mom_feed_rate [expr $mom_feed_rate * $mode_convert_scale]
            }
            if { [EQ_is_equal $mom_feed_rate 0.0] } {
               set mom_feed_rate [expr $mom_kin_rapid_feed_rate*$mom_sys_unit_conversion]
            }
            set dist [expr $dist-$mom_kin_reengage_distance]
            set mom_feed_rate_number [SET_FEEDRATE_NUMBER $dist $mom_feed_rate]
            FEEDRATE_SET
            MOM_suppress once fourth_axis fifth_axis
            PB_CMD_linear_move

           #
           #  Feed back to part at engage feed rate
           #
            MOM_suppress once fourth_axis fifth_axis
            if { $mom_feed_engage_value  > 0.0 } {
               set mom_feed_rate [expr $mom_feed_engage_value*$mom_sys_unit_conversion]
              #<01-07-10 wbh> Convert the feed rate from MMPR/IPR to MMPM/IPM
               if { [info exists mom_feed_engage_unit] && [string match "*pr" $mom_feed_engage_unit] } {
                  set mom_feed_rate [expr $mom_feed_rate * $mode_convert_scale]
               }
            } elseif { $mom_feed_cut_value  > 0.0 } {
               set mom_feed_rate [expr $mom_feed_cut_value*$mom_sys_unit_conversion]
              #<01-07-10 wbh> Convert the feed rate from MMPR/IPR to MMPM/IPM
               if { [info exists mom_feed_cut_unit] && [string match "*pr" $mom_feed_cut_unit] } {
                  set mom_feed_rate [expr $mom_feed_rate * $mode_convert_scale]
               }
            } else {
               set mom_feed_rate [expr 10.0*$mom_sys_unit_conversion]
            }

            VEC3_sub mom_pos mom_prev_pos del_pos
            set mom_feed_rate_number [SET_FEEDRATE_NUMBER $mom_kin_reengage_distance $mom_feed_rate]
            FEEDRATE_SET
            VMOV 3 mom_prev_pos mom_pos
            PB_CMD_linear_move

            VEC3_sub mom_pos save_pos del_pos
            set dist [VEC3_mag del_pos]
            set mom_feed_rate_number [SET_FEEDRATE_NUMBER $dist $mom_feed_rate]
            FEEDRATE_SET

            VMOV 5 save_pos mom_pos
            VMOV 5 save_prev_pos mom_prev_pos
            VMOV 2 save_out_angle_pos mom_out_angle_pos

         } else {
#
#  This section of code handles the case where there are two solutions to the tool axis inverse kinematics.
#  The post is forced to change from one solution to the other.  This causes a discontinuity in the tool path.
#  The post needs to retract, rotate to the new rotaries, then position back to the part using the alternate
#  solution.
#
            #
            #  Check for rotary axes in limits before retracting
            #
            set res [ANGLE_CHECK mom_prev_alt_pos(3) 4]
            if { $res == 1 } {
               set mom_out_angle_pos(0) [ROTSET $mom_prev_alt_pos(3) $mom_prev_rot_ang_4th $mom_kin_4th_axis_direction\
                                                $mom_kin_4th_axis_leader mom_sys_leader(fourth_axis)\
                                                $mom_kin_4th_axis_min_limit  $mom_kin_4th_axis_max_limit 1]
            } elseif { $res == 0 } {
               set mom_out_angle_pos(0) $mom_prev_alt_pos(3)
            } else {
               CATCH_WARNING "Not possible to position to alternate rotary axis positions. Gouging may result"
               VMOV 5 save_pos mom_pos

             return
            }

            set res [ANGLE_CHECK mom_prev_alt_pos(4) 5]
            if { $res == 1 } {
               set mom_out_angle_pos(1) [ROTSET $mom_prev_alt_pos(4) $mom_prev_rot_ang_5th $mom_kin_5th_axis_direction\
                                                $mom_kin_5th_axis_leader mom_sys_leader(fifth_axis)\
                                                $mom_kin_5th_axis_min_limit $mom_kin_5th_axis_max_limit 1]
            } elseif { $res == 0 } {
               set mom_out_angle_pos(1) $mom_prev_alt_pos(4)
            } else {
               CATCH_WARNING "Not possible to position to alternate rotary axis positions. Gouging may result"
               VMOV 5 save_pos mom_pos

             return
            }

            set mom_prev_pos(3) $mom_pos(3)
            set mom_prev_pos(4) $mom_pos(4)
            FEEDRATE_SET

            if { ![string compare "yes" $retract] } { PB_CMD_retract_move }
           #
           #  Move to alternate rotary position
           #
            set mom_pos(3) $mom_prev_alt_pos(3)
            set mom_pos(4) $mom_prev_alt_pos(4)
            set mom_prev_rot_ang_4th $mom_out_angle_pos(0)
            set mom_prev_rot_ang_5th $mom_out_angle_pos(1)
            VMOV 3 mom_prev_pos mom_pos
            FEEDRATE_SET
            PB_CMD_reposition_move

           #
           #  Position back to part at approach feed rate
           #
            set mom_prev_pos(3) $mom_pos(3)
            set mom_prev_pos(4) $mom_pos(4)
            for { set i 0 } { $i < 3 } { incr i } {
              set mom_pos($i) [expr $mom_prev_alt_pos($i)+$mom_kin_reengage_distance*$mom_sys_spindle_axis($i)]
            }
            MOM_suppress once fourth_axis fifth_axis
            set mom_feed_rate [expr $mom_feed_approach_value*$mom_sys_unit_conversion]

           #<01-07-10 wbh> Convert the feed rate from MMPR/IPR to MMPM/IPM
            if { [info exists mom_feed_approach_unit] && [string match "*pr" $mom_feed_approach_unit] } {
               set mom_feed_rate [expr $mom_feed_rate * $mode_convert_scale]
            }
            if { [EQ_is_equal $mom_feed_rate 0.0] } {
              set mom_feed_rate [expr $mom_kin_rapid_feed_rate * $mom_sys_unit_conversion]
            }
            set dist [expr $dist-$mom_kin_reengage_distance]
            set mom_feed_rate_number [SET_FEEDRATE_NUMBER $dist $mom_feed_rate]
            FEEDRATE_SET
            PB_CMD_linear_move

           #
           #  Feed back to part at engage feed rate
           #
            MOM_suppress once fourth_axis fifth_axis
            if { $mom_feed_engage_value  > 0.0 } {
               set mom_feed_rate [expr $mom_feed_engage_value*$mom_sys_unit_conversion]
              #<01-07-10 wbh> Convert the feed rate from MMPR/IPR to MMPM/IPM
               if { [info exists mom_feed_engage_unit] && [string match "*pr" $mom_feed_engage_unit] } {
                  set mom_feed_rate [expr $mom_feed_rate * $mode_convert_scale]
               }
            } elseif { $mom_feed_cut_value  > 0.0 } {
               set mom_feed_rate [expr $mom_feed_cut_value*$mom_sys_unit_conversion]
              #<01-07-10 wbh> Convert the feed rate from MMPR/IPR to MMPM/IPM
               if { [info exists mom_feed_cut_unit] && [string match "*pr" $mom_feed_cut_unit] } {
                  set mom_feed_rate [expr $mom_feed_rate * $mode_convert_scale]
               }
            } else {
              # ???
               set mom_feed_rate [expr 10.0*$mom_sys_unit_conversion]
            }

            set mom_feed_rate_number [SET_FEEDRATE_NUMBER $mom_kin_reengage_distance $mom_feed_rate]
            VMOV 3 mom_prev_alt_pos mom_pos
            FEEDRATE_SET
            PB_CMD_linear_move

            VEC3_sub mom_pos save_pos del_pos
            set dist [VEC3_mag del_pos]
            if { $dist <= 0.0 } { set dist $mom_kin_reengage_distance }
            set mom_feed_rate_number [SET_FEEDRATE_NUMBER $dist $mom_feed_rate]
            FEEDRATE_SET

            if { $roterr == 2 } {
               VMOV 5 mom_alt_pos mom_pos
            } else {
               VMOV 5 save_pos mom_pos
            }

           #<01-07-10 wbh> Reset the rotary sign
            RESET_ROTARY_SIGN $mom_pos(3) $mom_out_angle_pos(0) 3
            RESET_ROTARY_SIGN $mom_pos(4) $mom_out_angle_pos(1) 4

            set mom_out_angle_pos(0) [ROTSET $mom_pos(3) $mom_out_angle_pos(0) $mom_kin_4th_axis_direction\
                                             $mom_kin_4th_axis_leader mom_sys_leader(fourth_axis)\
                                             $mom_kin_4th_axis_min_limit $mom_kin_4th_axis_max_limit]
            set mom_out_angle_pos(1) [ROTSET $mom_pos(4) $mom_out_angle_pos(1) $mom_kin_5th_axis_direction\
                                             $mom_kin_5th_axis_leader mom_sys_leader(fifth_axis)\
                                             $mom_kin_5th_axis_min_limit $mom_kin_5th_axis_max_limit]

            MOM_reload_variable -a mom_out_angle_pos
            MOM_reload_variable -a mom_pos
            MOM_reload_variable -a mom_prev_pos
         }

         set mom_feed_rate $save_feedrate
         FEEDRATE_SET
      }
   }
}


#=============================================================
proc ROTATE_VECTOR { plane angle input_vector output_vector } {
#=============================================================
# Called by LOCK_AXIS & UNLOCK_AXIS

  upvar $output_vector v ; upvar $input_vector v1

   switch $plane {
      0 {
         set v(0) $v1(0)
         set v(1) [expr $v1(1)*cos($angle) - $v1(2)*sin($angle)]
         set v(2) [expr $v1(2)*cos($angle) + $v1(1)*sin($angle)]
      }

      1 {
         set v(0) [expr $v1(0)*cos($angle) + $v1(2)*sin($angle)]
         set v(1) $v1(1)
         set v(2) [expr $v1(2)*cos($angle) - $v1(0)*sin($angle)]
      }

      default {
         set v(0) [expr $v1(0)*cos($angle) - $v1(1)*sin($angle)]
         set v(1) [expr $v1(1)*cos($angle) + $v1(0)*sin($angle)]
         set v(2) $v1(2)
      }
   }
}


#=============================================================
proc ROTSET { angle prev_angle dir kin_leader sys_leader min max {tol_flag 0} } {
#=============================================================
#  This command will take an input angle and format for a specific machine.
#  It will also validate that the angle is within the specified limits of
#  machine.
#
#  angle        angle to be output.
#  prev_angle   previous angle output.  It should be mom_out_angle_pos
#  dir          can be either MAGNITUDE_DETERMINES_DIRECTION or
#               SIGN_DETERMINES_DIRECTION
#  kin_leader   leader (usually A, B or C) defined by Post Builder
#  sys_leader   leader that is created by ROTSET.  It could be "C-".
#  min          minimum degrees of travel for current axis
#  max          maximum degrees of travel for current axis
#
#  tol_flag     performance comparison with tolerance
#                 0 : No (default)
#                 1 : Yes
#
#
# - This command is called by the following functions:
#   RETRACT_ROTARY_AXIS, LOCK_AXIS_MOTION, LINEARIZE_LOCK_OUTPUT,
#   MOM_rotate, LINEARIZE_OUTPUT and MILL_TURN.
#
#=============================================================
# Revisions
# 02-25-2009 mzg - Added optional argument tol_flag to allow
#                  performing comparisions with tolerance
# 03-13-2012 gsl - (pb850) LIMIT_ANGLE should be called by using its return value
#                - Allow comparing max/min with tolerance
# 10-27-2015 gsl - Initialize mom_rotary_direction_4th & mom_rotary_direction_5th
# 04-25-2018 gsl - (pb12.02) Enhanced checking for rotation direction when "SIGN_DETERMINES_DIRECTION" is used.
#=============================================================

   upvar $sys_leader lead

  #
  #  Make sure angle is 0~360 to start with.
  #
   set angle [LIMIT_ANGLE $angle]
   set check_solution 0

   if { ![string compare "MAGNITUDE_DETERMINES_DIRECTION" $dir] } {

   #
   #  If magnitude determines direction and total travel is less than or equal
   #  to 360, we can assume there is at most one valid solution.  Find it and
   #  leave.  Check for the total travel being less than 360 and give a warning
   #  if a valid position cannot be found.
   #
      set travel [expr abs($max - $min)]

      if { $travel <= 360.0 } {

         set check_solution 1

      } else {

         if { $tol_flag == 0 } { ;# Exact comparison
            while { [expr abs([expr $angle - $prev_angle])] > 180.0 } {
               if { [expr $angle - $prev_angle] < -180.0 } {
                  set angle [expr $angle + 360.0]
               } elseif { [expr $angle - $prev_angle] > 180.0 } {
                  set angle [expr $angle - 360.0]
               }
            }
         } else { ;# Tolerant comparison
            while { [EQ_is_gt [expr abs([expr $angle - $prev_angle])] 180.0] } {
               if { [EQ_is_lt [expr $angle - $prev_angle] -180.0] } {
                  set angle [expr $angle + 360.0]
               } elseif { [EQ_is_gt [expr $angle - $prev_angle] 180.0] } {
                  set angle [expr $angle - 360.0]
               }
            }
         }
      }

     #<Apr-25-2018 gsl> Replaced with a command call
     #<03-13-12 gsl> Fit angle within limits
      set angle [MAXMIN_ANGLE $angle $max $min $tol_flag]

   } elseif { ![string compare "SIGN_DETERMINES_DIRECTION" $dir] } {

   #
   #  Sign determined direction determines whether the shortest distance is
   #  clockwise or counterclockwise.  If counterclockwise append a "-" sign
   #  to the address leader.
   #
      set check_solution 1

     #<09-15-09 wbh> If angle is negative, we add 360 to it instead of getting the absolute value of it.
      if { $angle < 0 } {
         set angle [expr $angle + 360]
      }

      set minus_flag 0
     # set angle [expr abs($angle)]  ;# This line was not in ROTSET of xzc post.

     #<Apr-25-2018 gsl> Should be abs(prev_angle)
      set prev_angle [expr abs($prev_angle)]

      set del [expr $angle - $prev_angle]
      if { $tol_flag == 0 } { ;# Exact comparison
         if { ($del < 0.0 && $del > -180.0) || $del > 180.0 } {
           # set lead "$kin_leader-"
            set minus_flag 1
         } else {
            set lead $kin_leader
         }
      } else { ;# Tolerant comparison
         if { ([EQ_is_lt $del 0.0] && [EQ_is_gt $del -180.0]) || [EQ_is_gt $del 180.0] } {
           # set lead "$kin_leader-"
            set minus_flag 1
         } else {
            set lead $kin_leader
         }
      }

     #<Apr-25-2018 gsl> Additional check if the move would cross the boundary, then reverse the direction.
      if { !$minus_flag && [EQ_is_lt $del 0.0] } {
         set n [expr int($prev_angle/360) + 1]
         if { [EQ_is_gt [expr $angle + $n*360.0] $max] } {
            set minus_flag 1
         }
      }


     #<04-27-11 wbh> 1819104 Check the rotary axis is 4th axis or 5th axis
      global mom_kin_4th_axis_leader mom_kin_5th_axis_leader
      global mom_rotary_direction_4th mom_rotary_direction_5th
      global mom_prev_rotary_dir_4th mom_prev_rotary_dir_5th

     #<Apr-25-2018 gsl> This logic may not determine is_4th properly,
     #                  since mom_kin_5th_axis_leader would always exist in a multi-axis post.
      set is_4th 1
      if { [info exists mom_kin_5th_axis_leader] && [string match "$mom_kin_5th_axis_leader" "$kin_leader"] } {
         set is_4th 0
      }

      if { ![info exists mom_rotary_direction_4th] } { set mom_rotary_direction_4th 1 }
      if { ![info exists mom_rotary_direction_5th] } { set mom_rotary_direction_5th 1 }

     #<09-15-09 wbh>
      if { $minus_flag && [EQ_is_gt $angle 0.0] } {
         set lead "$kin_leader-"

        #<04-27-11 wbh> Since the leader should add a minus, the rotary direction need be reset
         if { $is_4th } {
            set mom_rotary_direction_4th -1
         } else {
            set mom_rotary_direction_5th -1
         }
      } else {
        #<Apr-25-2018 gsl> Post (Tcl) needs to handle the opposite condition. The setting conveyed from the core processor is not reliable,
        #                  since the output angles are computed and produced by the post, core processor does not know about or keep track of.
         if { $is_4th } {
            set mom_rotary_direction_4th 1
         } else {
            set mom_rotary_direction_5th 1
         }
      }

     #<04-27-11 wbh> If the delta angle is 0 or 180, there has no need to change the rotary direction,
     #               we should reset the current direction with the previous direction
      if { [EQ_is_zero $del] || [EQ_is_equal $del 180.0] || [EQ_is_equal $del -180.0] } {
         if { $is_4th } {
            if { [info exists mom_prev_rotary_dir_4th] } {
               set mom_rotary_direction_4th $mom_prev_rotary_dir_4th
            }
         } else {
            if { [info exists mom_prev_rotary_dir_5th] } {
               set mom_rotary_direction_5th $mom_prev_rotary_dir_5th
            }
         }
      } else {
        # Set the previous direction
         if { $is_4th } {
            set mom_prev_rotary_dir_4th $mom_rotary_direction_4th
         } else {
            set mom_prev_rotary_dir_5th $mom_rotary_direction_5th
         }
      }

   } ;# "SIGN_DETERMINES_DIRECTION"


  #<03-13-12 gsl> Check solution
  #
  #  There are no alternate solutions.
  #  If the position is out of limits, give a warning and leave.
  #
   if { $check_solution } {
      if { $tol_flag == 1 } {
         if { [EQ_is_gt $angle $max] || [EQ_is_lt $angle $min] } {
            CATCH_WARNING "$kin_leader-axis is under minimum or over maximum. Assumed default."
         }
      } else {
         if { ($angle > $max) || ($angle < $min) } {
            CATCH_WARNING "$kin_leader-axis is under minimum or over maximum. Assumed default."
         }
      }
   }

 return $angle
}


#=============================================================
proc SET_FEEDRATE_NUMBER { dist feed } {
#=============================================================
# called by ROTARY_AXIS_RETRACT

#<03-13-08 gsl> FRN factor should not be used here! Leave it to PB_CMD_FEEDRATE_NUMBER
# global mom_sys_frn_factor

  global mom_kin_max_frn

  if { [EQ_is_zero $dist] } {
return $mom_kin_max_frn
  } else {
    set f [expr $feed / $dist ]
    if { [EQ_is_lt $f $mom_kin_max_frn] } {
return $f
    } else {
return $mom_kin_max_frn
    }
  }
}


#=============================================================
proc SET_LOCK { axis plane value } {
#=============================================================
# called by MOM_lock_axis

  upvar $axis a ; upvar $plane p ; upvar $value v

  global mom_kin_machine_type mom_lock_axis mom_lock_axis_plane mom_lock_axis_value
  global mom_warning_info

   set machine_type [string tolower $mom_kin_machine_type]
   switch $machine_type {
      4_axis_head       -
      4_axis_table      -
      3_axis_mill_turn  -
      mill_turn         { set mtype 4 }
      5_axis_dual_table -
      5_axis_dual_head  -
      5_axis_head_table { set mtype 5 }
      default {
         set mom_warning_info "Set lock only vaild for 4 and 5 axis machines"
return "error"
      }
   }

   # Check the locked rotary axis.
   # If the rotary axis is the locked axis, it must be the 4th axis for 4-axis machine,
   # or the 5th axis for 5-axis machine.
   if { ![CHECK_LOCK_ROTARY_AXIS $mom_lock_axis $mtype] } {
      set mom_warning_info "Specified rotary axis is invalid as the lock axis"
      return "error"
   }

   set p -1

   global mom_sys_lock_arc_save
   global mom_kin_arc_output_mode

   switch $mom_lock_axis {
      OFF {
         if { [info exists mom_sys_lock_arc_save] } {
             set mom_kin_arc_output_mode $mom_sys_lock_arc_save
             unset mom_sys_lock_arc_save
             MOM_reload_kinematics
         }
         return "OFF"
      }
      XAXIS {
         set a 0
         switch $mom_lock_axis_plane {
            XYPLAN {
               set v [LOCK_AXIS_SUB $a]
               set p 2
            }
            YZPLAN {
               set mom_warning_info "Invalid plane for lock axis"
               return "error"
            }
            ZXPLAN {
               set v [LOCK_AXIS_SUB $a]
               set p 1
            }
            NONE {
               if { $mtype == 5 } {
                  set mom_warning_info "Must specify lock axis plane for 5 axis machine"
                  return "error"
               } else {
                  set v [LOCK_AXIS_SUB $a]
               }
            }
         }
      }
      YAXIS {
         set a 1
         switch $mom_lock_axis_plane {
            XYPLAN {
               set v [LOCK_AXIS_SUB $a]
               set p 2
            }
            YZPLAN {
               set v [LOCK_AXIS_SUB $a]
               set p 0
            }
            ZXPLAN {
               set mom_warning_info "Invalid plane for lock axis"
               return "error"
            }
            NONE {
               if { $mtype == 5 } {
                  set mom_warning_info "Must specify lock axis plane for 5 axis machine"
                  return "error"
               } else {
                  set v [LOCK_AXIS_SUB $a]
               }
            }
         }
      }
      ZAXIS {
         set a 2
         switch $mom_lock_axis_plane {
            YZPLAN {
               set v [LOCK_AXIS_SUB $a]
               set p 0
            }
            ZXPLAN {
               set v [LOCK_AXIS_SUB $a]
               set p 1
            }
            XYPLAN {
               set mom_warning_info "Invalid plane for lock axis"
               return "error"
            }
            NONE {
               if { $mtype == 5 } {
                  set mom_warning_info "Must specify lock axis plane for 5 axis machine"
                  return "error"
               } else {
                  set v [LOCK_AXIS_SUB $a]
               }
            }
         }
      }
      FOURTH {
         set a 3
         set v [LOCK_AXIS_SUB $a]
      }
      FIFTH {
         set a 4
         set v [LOCK_AXIS_SUB $a]
      }
      AAXIS {
         set a [AXIS_SET $mom_lock_axis]
         set v [LOCK_AXIS_SUB $a]
      }
      BAXIS {
         set a [AXIS_SET $mom_lock_axis]
         set v [LOCK_AXIS_SUB $a]
      }
      CAXIS {
         set a [AXIS_SET $mom_lock_axis]
         set v [LOCK_AXIS_SUB $a]
      }
   }

   if { ![info exists mom_sys_lock_arc_save] } {
      set mom_sys_lock_arc_save $mom_kin_arc_output_mode
   }

   set mom_kin_arc_output_mode "LINEAR"
   MOM_reload_kinematics

return "ON"
}


#=============================================================
proc SOLVE_QUADRATIC { coeff rcomp icomp status degree } {
#=============================================================
# called by CALC_SPHERICAL_RETRACT_POINT

   upvar $coeff c ; upvar $rcomp rc ; upvar $icomp ic
   upvar $status st ; upvar $degree deg

   set st 1
   set deg 0
   set rc(0) 0.0 ; set rc(1) 0.0
   set ic(0) 0.0 ; set ic(1) 0.0
   set mag [VEC3_mag c]
   if { [EQ_is_zero $mag] } { return 0 }

   set acoeff [expr $c(2)/$mag]
   set bcoeff [expr $c(1)/$mag]
   set ccoeff [expr $c(0)/$mag]

   if { ![EQ_is_zero $acoeff] } {
      set deg 2
      set denom [expr $acoeff*2.]
      set dscrm [expr $bcoeff*$bcoeff - $acoeff*$ccoeff*4.0]
      if { [EQ_is_zero $dscrm] } {
         set dsqrt1 0.0
      } else {
         set dsqrt1 [expr sqrt(abs($dscrm))]
      }
      if { [EQ_is_ge $dscrm 0.0] } {
         set rc(0) [expr (-$bcoeff + $dsqrt1)/$denom]
         set rc(1) [expr (-$bcoeff - $dsqrt1)/$denom]
         set st 3
         return 2
      } else {
         set rc(0) [expr -$bcoeff/$denom]
         set rc(1) $rc(0)
         set ic(0) [expr $dsqrt1/$denom]
         set ic(1) $ic(0)
         set st 2
         return 0
      }
   } elseif { ![EQ_is_zero $bcoeff] } {
      set st 3
      set deg 1
      set rc(0) [expr -$ccoeff/$bcoeff]
      return 1
   } elseif { [EQ_is_zero $ccoeff] } {
      return 0
   } else {
      return 0
   }
}


#=============================================================
proc STR_MATCH { VAR str {out_warn 0} } {
#=============================================================
# This command will match a variable with a given string.
#
# - Users can set the optional flag "out_warn" to "1" to produce
#   warning message when the variable is not defined in the scope
#   of the caller of this function.
#
   upvar $VAR var

   if { [info exists var] && [string match "$str" "$var"] } {
return 1
   } else {
      if { $out_warn && ![info exists var] } {
         CATCH_WARNING "Variable $VAR is not defined in \"[lindex [info level -1] 0]\"!"
      }
return 0
   }
}


#=============================================================
proc TRACE { {up_level 0} } {
#=============================================================
# "up_level" to be a negative integer
#
   set start_idx 1

   set str ""
   set level [expr [info level] - int(abs($up_level))]

   for { set i $start_idx } { $i <= $level } { incr i } {
      if { $i < $level } {
         set str "${str}[lindex [info level $i] 0]\n"
      } else {
         set str "${str}[lindex [info level $i] 0]"
      }
   }

return $str
}


#=============================================================
proc UNLOCK_AXIS { locked_point unlocked_point } {
#=============================================================
# called by LINEARIZE_LOCK_MOTION
#
# (pb903)
# 04-16-14 gsl - Account for offsets resulted from right-angled head attachment
# 09-09-15 ljt - Replace mom_kin_4/5th_axis_center_offset with mom_kin_4/5th_axis_point
#
# (pb1101)
# 08-11-16 gsl - Correct error for removing offsets
#

   upvar $locked_point in_pos ; upvar $unlocked_point out_pos

   global mom_sys_lock_plane
   global mom_sys_linear_axis_index_1
   global mom_sys_linear_axis_index_2
   global mom_sys_rotary_axis_index
   global mom_kin_4th_axis_center_offset
   global mom_kin_5th_axis_center_offset
   global mom_kin_4th_axis_point
   global mom_kin_5th_axis_point
   global mom_kin_machine_type
   global mom_origin
   global DEG2RAD


  #<04-16-2014 gsl> Add offsets of angled-head attachment to input point
   VMOV 5 in_pos ip
   ACCOUNT_HEAD_OFFSETS ip 1


   #<09-Sep-2015 ljt> Add offsets of 4/5th axis rotary center
   VMOV 3 ip temp
   if { [CMD_EXIST MOM_validate_machine_model] \
        && [string match "TRUE" [MOM_validate_machine_model]] } {

      if { [string match "5_axis_*table" $mom_kin_machine_type] && [info exists mom_kin_5th_axis_point] } {

         VEC3_sub temp mom_kin_5th_axis_point temp

      } elseif { ( [string match "4_axis_table" $mom_kin_machine_type] || [string match "*mill_turn" $mom_kin_machine_type] )\
                 && [info exists mom_kin_4th_axis_point] } {

         VEC3_sub temp mom_kin_4th_axis_point temp
      }

   } else {

      if { [info exists mom_origin] } {
         VEC3_sub temp mom_origin temp
      }

      if { [info exists mom_kin_4th_axis_center_offset] } {
         VEC3_add temp mom_kin_4th_axis_center_offset temp
      }

      if { [info exists mom_kin_5th_axis_center_offset] } {
         VEC3_add temp mom_kin_5th_axis_center_offset temp
      }
   }

   set op(3) $ip(3)
   set op(4) $ip(4)

   set ang [expr $op($mom_sys_rotary_axis_index)*$DEG2RAD]
   ROTATE_VECTOR $mom_sys_lock_plane $ang temp op

   set op($mom_sys_rotary_axis_index) 0.0


   #<09-Sep-2015 ljt> Remove offsets of 4/5th axis rotary center
   if { [CMD_EXIST MOM_validate_machine_model] &&\
        [string match "TRUE" [MOM_validate_machine_model]] } {

      if { [string match "5_axis_*table" $mom_kin_machine_type] && [info exists mom_kin_5th_axis_point] } {

         VEC3_add op mom_kin_5th_axis_point op

      } elseif { ( [string match "4_axis_table" $mom_kin_machine_type] || [string match "*mill_turn" $mom_kin_machine_type] ) && \
                 [info exists mom_kin_4th_axis_point] } {

         VEC3_add op mom_kin_4th_axis_point op
      }

   } else {

     #<Aug-11-2016> Reverse next 3 operations
      if { [info exists mom_origin] } {
         VEC3_sub op mom_origin op
      }

      if { [info exists mom_kin_4th_axis_center_offset] } {
         VEC3_add op mom_kin_4th_axis_center_offset op
      }

      if { [info exists mom_kin_5th_axis_center_offset] } {
         VEC3_add op mom_kin_5th_axis_center_offset op
      }
   }


  #<04-16-2014 gsl> Remove offsets of angled-head attachment from output point
   ACCOUNT_HEAD_OFFSETS op 0
   VMOV 5 op out_pos
}


#=============================================================
proc UNLOCK_AXIS__pb901 { locked_point unlocked_point } {
#=============================================================
# called by LINEARIZE_LOCK_MOTION

   upvar $locked_point ip ; upvar $unlocked_point op

   global mom_sys_lock_plane
   global mom_sys_linear_axis_index_1
   global mom_sys_linear_axis_index_2
   global mom_sys_rotary_axis_index
   global mom_kin_4th_axis_center_offset
   global mom_kin_5th_axis_center_offset
   global DEG2RAD


   if { [info exists mom_kin_4th_axis_center] } {
       VEC3_add ip mom_kin_4th_axis_center_offset temp
   } else {
       set temp(0) $ip(0)
       set temp(1) $ip(1)
       set temp(2) $ip(2)
   }
   if { [info exists mom_kin_5th_axis_center_offset] } {
      VEC3_add temp mom_kin_5th_axis_center_offset temp
   }

   set op(3) $ip(3)
   set op(4) $ip(4)

   set ang [expr $op($mom_sys_rotary_axis_index)*$DEG2RAD]
   ROTATE_VECTOR $mom_sys_lock_plane $ang temp op

   set op($mom_sys_rotary_axis_index) 0.0

   if { [info exists mom_kin_4th_axis_center_offset] } {
      VEC3_sub op mom_kin_4th_axis_center_offset op
   }
   if { [info exists mom_kin_5th_axis_center_offset] } {
      VEC3_sub op mom_kin_5th_axis_center_offset op
   }
}


#=============================================================
proc UNSET_VARS { args } {
#=============================================================
# Inputs: List of variable names
#

   if { [llength $args] == 0 } {
return
   }

   foreach VAR $args {

      set VAR [string trim $VAR]
      if { $VAR != "" } {

         upvar $VAR var

         if { [array exists var] } {
            if { [expr $::tcl_version > 8.1] } {
               array unset var
            } else {
               foreach a [array names var] {
                  if { [info exists var($a)] } {
                     unset var($a)
                  }
               }
               unset var
            }
         }

         if { [info exists var] } {
            unset var
         }

      }
   }
}


#=============================================================
proc VALIDATE_MOTION { } {
#=============================================================
# To be called by PB_CMD_kin_before_motion

   if [CMD_EXIST PB_CMD__validate_motion] {
return [PB_CMD__validate_motion]
   } else {
      # Assume OK, when no validation is done.
return 1
   }
}


#-------------------------------------------------------------
proc VEC3_find_plane_angle { VEC VECP } {
#-------------------------------------------------------------
# Compute angle of a vector w.r.t the plane of a given normal.
# It returns angle in degrees.
#
# Jan 24, 2013 gsl -
#
   upvar $VEC  v
   upvar $VECP p ;# Plane normal vector

   expr { 90.0 - $::RAD2DEG * acos([VEC3_dot v p]) }
}


#-------------------------------------------------------------
proc VEC3_find_principal_angle { VEC plane } {
#-------------------------------------------------------------
# Compute angle of a vector w.r.t a given principal plane.
# It returns angle in degrees.
#
# Jan 24, 2013 gsl -
#
   upvar $VEC v

  # Assign principal plane axis vector
   switch $plane {
      XY {
        # set p(0) 0.0;  set p(1) 0.0;  set p(2) 1.0
         set r1 $v(2);  set r2 $v(0);  set r3 $v(1)
      }
      YZ {
        # set p(0) 1.0;  set p(1) 0.0;  set p(2) 0.0
         set r1 $v(0);  set r2 $v(1);  set r3 $v(2)
      }
      ZX {
        # set p(0) 0.0;  set p(1) 1.0;  set p(2) 0.0
         set r1 $v(1);  set r2 $v(2);  set r3 $v(0)
      }
      default { }
   }

   expr { $::RAD2DEG*atan2($r1,[expr sqrt($r2*$r2 + $r3*$r3)]) }
}


#=============================================================
proc VECTOR_ROTATE { axis angle input_vector output_vector } {
#=============================================================
# This command is used to rotating a vector about arbitrary axis.
   upvar $axis r; upvar $input_vector input ; upvar $output_vector output
   #set up matrix to rotate about an arbitrary axis
   set m(0) [expr $r(0)*$r(0)*(1-cos($angle))+cos($angle)]
   set m(1) [expr $r(0)*$r(1)*(1-cos($angle))-$r(2)*sin($angle)]
   set m(2) [expr $r(0)*$r(2)*(1-cos($angle))+$r(1)*sin($angle)]
   set m(3) [expr $r(0)*$r(1)*(1-cos($angle))+$r(2)*sin($angle)]
   set m(4) [expr $r(1)*$r(1)*(1-cos($angle))+cos($angle)]
   set m(5) [expr $r(1)*$r(2)*(1-cos($angle))-$r(0)*sin($angle)]
   set m(6) [expr $r(0)*$r(2)*(1-cos($angle))-$r(1)*sin($angle)]
   set m(7) [expr $r(1)*$r(2)*(1-cos($angle))+$r(0)*sin($angle)]
   set m(8) [expr $r(2)*$r(2)*(1-cos($angle))+cos($angle)]
   MTX3_vec_multiply input m output
}


#=============================================================
proc VMOV { n p1 p2 } {
#=============================================================
  upvar $p1 v1 ; upvar $p2 v2

   for { set i 0 } { $i < $n } { incr i } {
      set v2($i) $v1($i)
   }
}


#=============================================================
proc WORKPLANE_SET { } {
#=============================================================
   global mom_cycle_spindle_axis
   global mom_sys_spindle_axis
   global traverse_axis1 traverse_axis2

   if { ![info exists mom_sys_spindle_axis] } {
      set mom_sys_spindle_axis(0) 0.0
      set mom_sys_spindle_axis(1) 0.0
      set mom_sys_spindle_axis(2) 1.0
   }

   if { ![info exists mom_cycle_spindle_axis] } {
      set x $mom_sys_spindle_axis(0)
      set y $mom_sys_spindle_axis(1)
      set z $mom_sys_spindle_axis(2)

      if { [EQ_is_zero $y] && [EQ_is_zero $z] } {
         set mom_cycle_spindle_axis 0
      } elseif { [EQ_is_zero $x] && [EQ_is_zero $z] } {
         set mom_cycle_spindle_axis 1
      } else {
         set mom_cycle_spindle_axis 2
      }
   }

   if { $mom_cycle_spindle_axis == 2 } {
      set traverse_axis1 0 ; set traverse_axis2 1
   } elseif { $mom_cycle_spindle_axis == 0 } {
      set traverse_axis1 1 ; set traverse_axis2 2
   } elseif { $mom_cycle_spindle_axis == 1 } {
      set traverse_axis1 0 ; set traverse_axis2 2
   }
}


#-------------------------------------------------------------
proc cosD { a } {
#-------------------------------------------------------------
return [expr cos( $::DEG2RAD * $a )]
}


#-------------------------------------------------------------
proc deg2rad { a } {
#-------------------------------------------------------------
return [expr $a * $::DEG2RAD]
}


#-------------------------------------------------------------
proc rad2deg { a } {
#-------------------------------------------------------------
return [expr $a * $::RAD2DEG]
}


#-------------------------------------------------------------
proc sinD { a } {
#-------------------------------------------------------------
return [expr sin( $::DEG2RAD * $a )]
}


#=============================================================
proc PB_load_alternate_unit_settings { } {
#=============================================================
   global mom_output_unit mom_kin_output_unit

  # Skip this function when output unit agrees with post unit.
   if { ![info exists mom_output_unit] } {
      set mom_output_unit $mom_kin_output_unit
return
   } elseif { ![string compare $mom_output_unit $mom_kin_output_unit] } {
return
   }


   global mom_event_handler_file_name

  # Set unit conversion factor
   if { ![string compare $mom_output_unit "MM"] } {
      set factor 25.4
   } else {
      set factor [expr 1/25.4]
   }

  # Define unit dependent variables list
   set unit_depen_var_list [list mom_kin_x_axis_limit mom_kin_y_axis_limit mom_kin_z_axis_limit \
                                 mom_kin_pivot_gauge_offset mom_kin_ind_to_dependent_head_x \
                                 mom_kin_ind_to_dependent_head_z]

   set unit_depen_arr_list [list mom_kin_4th_axis_center_offset \
                                 mom_kin_5th_axis_center_offset \
                                 mom_kin_machine_zero_offset \
                                 mom_kin_4th_axis_point \
                                 mom_kin_5th_axis_point \
                                 mom_sys_home_pos]

  # Load unit dependent variables
   foreach var $unit_depen_var_list {
      if { ![info exists $var] } {
         global $var
      }
      if { [info exists $var] } {
         set $var [expr [set $var] * $factor]
         MOM_reload_variable $var
      }
   }

   foreach var $unit_depen_arr_list {
      if { ![info exists $var] } {
         global $var
      }
      foreach item [array names $var] {
         if { [info exists ${var}($item)] } {
            set ${var}($item) [expr [set ${var}($item)] * $factor]
         }
      }

      MOM_reload_variable -a $var
   }


  # Source alternate unit post fragment
   uplevel #0 {
      global mom_sys_alt_unit_post_name
      set alter_unit_post_name \
          "[file join [file dirname $mom_event_handler_file_name] [file rootname $mom_sys_alt_unit_post_name]].tcl"

      if { [file exists $alter_unit_post_name] } {
         source "$alter_unit_post_name"
      }
      unset alter_unit_post_name
   }

   if { [llength [info commands PB_load_address_redefinition]] > 0 } {
      PB_load_address_redefinition
   }

   MOM_reload_kinematics
}


if [info exists mom_sys_start_of_program_flag] {
   if [llength [info commands PB_CMD_kin_start_of_program] ] {
      PB_CMD_kin_start_of_program
   }
} else {
   set mom_sys_head_change_init_program 1
   set mom_sys_start_of_program_flag 1
}


