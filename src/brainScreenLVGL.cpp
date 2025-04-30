#include "main.h"
#include "config.h"
#include "liblvgl/llemu.h"
#include <string>

int autonState = 0;

/// @brief 
void screen(){
	while(1){
		//lv_init();
		//Set variable for color style
		static lv_style_t Genesis_blue; //Creates a variable name "Geneis_blue" that is of lv_style_t type
		lv_style_init(&Genesis_blue);//Function takes memory address of "Genesis_blue"
		lv_style_set_bg_opa(&Genesis_blue, LV_OPA_COVER);
		lv_style_set_bg_color(&Genesis_blue, lv_color_hex(0x8bb8e3));
		lv_style_set_radius(&Genesis_blue, 7);

		static lv_style_t Genesis_yellow; //Creates a variable name "Geneis_blue" that is of lv_style_t type
		lv_style_init(&Genesis_yellow);//Function takes memory address of "Genesis_blue"
		lv_style_set_bg_opa(&Genesis_yellow, LV_OPA_COVER);
		lv_style_set_bg_color(&Genesis_yellow, lv_color_hex(0xf3c926));
		lv_style_set_radius(&Genesis_yellow, 7);
		
		static lv_style_t Genesis_bg_color; //Creates a variable name "Geneis_blue" that is of lv_style_t type
		lv_style_init(&Genesis_bg_color);//Function takes memory address of "Genesis_blue"
		lv_style_set_bg_opa( &Genesis_bg_color, LV_OPA_COVER);
		lv_style_set_bg_color(& Genesis_bg_color, lv_color_hex(0xf3c926));
		lv_style_set_radius( &Genesis_bg_color, 7);

		/*Init the pressed style*/
		static lv_style_t style_pr;
		lv_style_init(&style_pr);
		lv_style_set_bg_color(&style_pr, lv_palette_darken(LV_PALETTE_BLUE, 3));



		// myButton = lv_btn_create(lv_scr_act()); //create button, lv_scr_act() is deafult screen object
		// lv_obj_add_event_cb(myButton, btn_click_action, LV_EVENT_ALL, NULL); //set function to be called on button click
		// lv_obj_add_style(myButton, &style_pr, LV_STATE_PRESSED);
		// lv_obj_set_size(myButton, 200, 50); //set the button size
		// lv_obj_align(myButton, LV_ALIGN_TOP_LEFT, 10, 10); //set the position to top mid

		// myButtonLabel = lv_label_create(myButton); //create label and puts it inside of the button
		// lv_label_set_text(myButtonLabel, "Click the Button"); //sets label text
		// lv_obj_center(myButtonLabel);

		// lv_obj_t *img = lv_img_create(lv_scr_act());
		// lv_img_set_src(img, &logo_78181A);
		// lv_obj_set_pos(img,20,20);

		// lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x737373),LV_PART_MAIN);

		lv_obj_t *bar1bg = lv_bar_create(lv_scr_act());//pointer refers too a variable that stores memeroy address
		lv_obj_remove_style_all(bar1bg);
		lv_obj_add_style(bar1bg, &Genesis_yellow, LV_PART_INDICATOR);
		lv_obj_set_size(bar1bg, 158 ,13);
		lv_obj_set_pos (bar1bg, 133, 25);
		lv_bar_set_value(bar1bg, 100, LV_ANIM_OFF);

		lv_obj_t *bar1 = lv_bar_create(lv_scr_act());//pointer refers too a variable that stores memeroy address
		lv_obj_remove_style_all(bar1);
		lv_obj_add_style(bar1, &Genesis_blue, LV_PART_INDICATOR);
		lv_obj_set_size(bar1, 158 ,13);
		lv_obj_set_pos (bar1, 133, 25);
		lv_bar_set_value(bar1, 0, LV_ANIM_ON);

		//2nd bar 
		lv_obj_t *bar2bg = lv_bar_create(lv_scr_act());//pointer refers too a variable that stores memeroy address
		lv_obj_remove_style_all(bar2bg);
		lv_obj_add_style(bar2bg, &Genesis_yellow, LV_PART_INDICATOR);
		lv_obj_set_size(bar2bg, 158 ,13);
		lv_obj_set_pos (bar2bg, 133, 60);
		lv_bar_set_value(bar2bg, 100, LV_ANIM_OFF);

		lv_obj_t *bar2 = lv_bar_create(lv_scr_act());//pointer refers too a variable that stores memeroy address
		lv_obj_remove_style_all(bar2);
		lv_obj_add_style(bar2, &Genesis_blue, LV_PART_INDICATOR);
		lv_obj_set_size(bar2, 158 ,13);
		lv_obj_set_pos (bar2, 133, 60);
		lv_bar_set_value(bar2, 66, LV_ANIM_ON);

		//3rd bar 
		lv_obj_t *bar3bg = lv_bar_create(lv_scr_act());//pointer refers too a variable that stores memeroy address
		lv_obj_remove_style_all(bar3bg);
		lv_obj_add_style(bar3bg, &Genesis_yellow, LV_PART_INDICATOR);
		lv_obj_set_size(bar3bg, 158 ,13);
		lv_obj_set_pos (bar3bg, 133, 95);
		lv_bar_set_value(bar3bg, 100, LV_ANIM_OFF);

		lv_obj_t *bar3 = lv_bar_create(lv_scr_act());//pointer refers too a variable that stores memeroy address
		lv_obj_remove_style_all(bar3);
		lv_obj_add_style(bar3, &Genesis_blue, LV_PART_INDICATOR);
		lv_obj_set_size(bar3, 158 ,13);
		lv_obj_set_pos (bar3, 133, 95);
		lv_bar_set_value(bar3, 80, LV_ANIM_ON);
	
		//4th bar

		lv_obj_t *bar4bg = lv_bar_create(lv_scr_act());//pointer refers too a variable that stores memeroy address
		lv_obj_remove_style_all(bar4bg);
		lv_obj_add_style(bar4bg, &Genesis_yellow, LV_PART_INDICATOR);
		lv_obj_set_size(bar4bg, 158 ,13);
		lv_obj_set_pos (bar4bg, 133, 130);
		lv_bar_set_value(bar4bg, 100, LV_ANIM_OFF);

		lv_obj_t *bar4 = lv_bar_create(lv_scr_act());//pointer refers too a variable that stores memeroy address
		lv_obj_remove_style_all(bar4);
		lv_obj_add_style(bar4, &Genesis_blue, LV_PART_MAIN);
		lv_obj_set_size(bar4, 158 ,13);
		lv_obj_set_pos (bar4, 133, 130);
		lv_bar_set_value(bar4, 50, LV_ANIM_ON);

		//Motor Temperature
		lv_obj_t *arcbg = lv_arc_create(lv_scr_act());
		lv_obj_remove_style(arcbg, NULL, LV_PART_KNOB);
		lv_obj_add_style(arcbg,&Genesis_yellow ,LV_PART_INDICATOR);//this line aint working for some reason
		lv_arc_set_rotation(arcbg, 135);
		lv_arc_set_bg_angles(arcbg, 0, 270);
		lv_arc_set_value(arcbg, 100);
		lv_obj_set_pos (arcbg, 140, 160);
		lv_obj_set_size(arcbg, 60 ,59);

		lv_obj_t *arcbg_a = lv_arc_create(lv_scr_act());
		lv_obj_remove_style(arcbg_a, NULL, LV_PART_KNOB);
		lv_obj_add_style(arcbg_a,&Genesis_yellow ,LV_PART_INDICATOR);//this line aint working for some reason
		lv_arc_set_rotation(arcbg_a, 135);
		lv_arc_set_bg_angles(arcbg_a, 0, 270);
		lv_arc_set_value(arcbg_a, 100);
		lv_obj_set_pos (arcbg_a, 230, 160);
		lv_obj_set_size(arcbg_a, 60 ,59);
		
		lv_obj_t *arcbg_b = lv_arc_create(lv_scr_act());
		lv_obj_remove_style(arcbg_b, NULL, LV_PART_KNOB);
		lv_obj_add_style(arcbg_b,&Genesis_yellow ,LV_PART_INDICATOR);//this line aint working for some reason
		lv_arc_set_rotation(arcbg_b, 135);
		lv_arc_set_bg_angles(arcbg_b, 0, 270);
		lv_arc_set_value(arcbg_b, 100);
		lv_obj_set_pos (arcbg_b, 320, 160);
		lv_obj_set_size(arcbg_b, 60 ,59);

		lv_obj_t *arcbg_c = lv_arc_create(lv_scr_act());
		lv_obj_remove_style(arcbg_c, NULL, LV_PART_KNOB);
		lv_obj_add_style(arcbg_c,&Genesis_blue ,LV_PART_INDICATOR);//this line aint working for some reason
		lv_arc_set_rotation(arcbg_c, 135);
		lv_arc_set_bg_angles(arcbg_c, 0, 270);
		lv_arc_set_value(arcbg_c, 100);
		lv_obj_set_pos (arcbg_c, 410, 160);
		lv_obj_set_size(arcbg_c, 60 ,59);

		//Roller: --------------------------------------------------------
		lv_obj_t * autonroller = lv_roller_create(lv_scr_act());
		lv_obj_set_pos(autonroller, 320, 38);
		lv_roller_set_options(autonroller,
							"Testing\n"
							"Solo Red"
							"Red- Qual\n"
							"Red+ Qual\n"
							"Solo Blue"
							"Blue- Qual\n"
							"Blue+ Qual\n"
							"Red- Elim\n"
							"Red+ Elim\n"
							"Blue- Elim\n"
							"Blue+ Elim\n"
							"Skills\n",
							LV_ROLLER_MODE_INFINITE);
		lv_obj_add_style(autonroller,&Genesis_blue ,LV_PART_INDICATOR);
		lv_obj_set_size(autonroller, 150 ,250);
		lv_roller_set_visible_row_count(autonroller, 5);

		if (autonState == 0) lv_roller_set_selected(autonroller,0,LV_ANIM_ON);
		else if (autonState == 1) lv_roller_set_selected(autonroller,1,LV_ANIM_ON);
		else if (autonState == 2) lv_roller_set_selected(autonroller,2,LV_ANIM_ON);
		else if (autonState == 3) lv_roller_set_selected(autonroller,3,LV_ANIM_ON);
		else if (autonState == 4) lv_roller_set_selected(autonroller,4,LV_ANIM_ON);
		else if (autonState == 5) lv_roller_set_selected(autonroller,5,LV_ANIM_ON);
		else if (autonState == 6) lv_roller_set_selected(autonroller,6,LV_ANIM_ON);
		else if (autonState == 7) lv_roller_set_selected(autonroller,7,LV_ANIM_ON);
		else if (autonState == 8) lv_roller_set_selected(autonroller,8,LV_ANIM_ON);
		else if (autonState == 9) lv_roller_set_selected(autonroller,9,LV_ANIM_ON);
		else if (autonState == 10) lv_roller_set_selected(autonroller,10,LV_ANIM_ON);
		else if (autonState == 11) lv_roller_set_selected(autonroller,11,LV_ANIM_ON);

		if (lv_roller_get_selected(autonroller) == 0) autonState = 0;
		else if(lv_roller_get_selected(autonroller) == 1) autonState = 1;
		else if(lv_roller_get_selected(autonroller) == 2) autonState = 2;
		else if(lv_roller_get_selected(autonroller) == 3) autonState = 3;
		else if(lv_roller_get_selected(autonroller) == 4) autonState = 4;
		else if(lv_roller_get_selected(autonroller) == 5) autonState = 5;
		else if(lv_roller_get_selected(autonroller) == 6) autonState = 6;
		else if(lv_roller_get_selected(autonroller) == 7) autonState = 7;
		else if(lv_roller_get_selected(autonroller) == 8) autonState = 8;
		else if(lv_roller_get_selected(autonroller) == 9) autonState = 9;
		else if(lv_roller_get_selected(autonroller) == 10) autonState = 10;
		else if(lv_roller_get_selected(autonroller) == 11) autonState = 11;
		//Battery and radio signal: ------------------------------------

		//Background of Battery
		lv_obj_t *Bat_rec_bg = lv_bar_create(lv_scr_act());
		lv_obj_set_style_radius(Bat_rec_bg, 0, LV_PART_INDICATOR);
		lv_obj_set_pos(Bat_rec_bg, 25, 93);
		lv_obj_set_size(Bat_rec_bg, 75, 24);
		lv_bar_set_value(Bat_rec_bg, 100, LV_ANIM_OFF);
		lv_obj_add_style(Bat_rec_bg,&Genesis_yellow ,LV_PART_INDICATOR);
		lv_obj_set_style_radius(Bat_rec_bg , 0, 0);

		//Overlap the background (color blue_)
		lv_obj_t *Bat_rec_indicator = lv_bar_create(lv_scr_act());
		lv_obj_set_style_radius(Bat_rec_indicator, 0, LV_PART_INDICATOR);
		lv_obj_set_pos(Bat_rec_indicator, 25, 93);
		lv_obj_set_size(Bat_rec_indicator, 75, 26);
		lv_bar_set_value(Bat_rec_indicator, pros::battery::get_capacity()*(1/12.8)*100, LV_ANIM_ON);
		lv_obj_add_style(Bat_rec_indicator,&Genesis_blue ,LV_PART_INDICATOR);
		lv_obj_set_style_radius(Bat_rec_indicator, 0, 0);

		//Radio rectangle
		// (bat cover 1) = long rectangle, bat cover 2 = short rectangle
		lv_obj_t *Bat_cover1 = lv_bar_create(lv_scr_act());
		lv_obj_set_style_radius(Bat_cover1, 0, LV_PART_INDICATOR);
		lv_obj_set_pos(Bat_cover1, 98, 93); 
		lv_obj_set_size(Bat_cover1, 6, 26);
		lv_bar_set_value(Bat_cover1, 100, LV_ANIM_OFF);
		lv_obj_add_style(Bat_cover1,&Genesis_bg_color ,LV_PART_INDICATOR);
		lv_obj_set_style_radius(Bat_cover1 , 0, 0);

		lv_obj_t *Bat_cover2 = lv_bar_create(lv_scr_act());
		lv_obj_set_style_radius(Bat_cover2, 0, LV_PART_INDICATOR);
		lv_obj_set_pos(Bat_cover2, 102, 100); //92,101
		lv_obj_set_size(Bat_cover2, 8, 10);
		lv_bar_set_value(Bat_cover2, 100, LV_ANIM_OFF);
		lv_obj_add_style(Bat_cover2,&Genesis_bg_color ,LV_PART_INDICATOR);
		lv_obj_set_style_radius(Bat_cover2 , 0, 0);

		//Inertial Sensor degree
		lv_obj_t *Outer_arc = lv_arc_create(lv_scr_act());
		lv_arc_set_bg_angles(Outer_arc, 0, 360);
		// lv_obj_add_style(Outer_arc,&Genesis_blue,0); //genesis yellow
		lv_obj_add_style(Outer_arc, &Genesis_blue, LV_PART_KNOB);
		lv_obj_set_size(Outer_arc, 100, 100);
		lv_obj_set_pos(Outer_arc , 10, 135);


		//not needed ---------------
		// lv_obj_t *Inner_background = lv_obj_create(lv_scr_act());
		// lv_obj_set_size(Inner_background , 42, 42);
		// lv_obj_set_pos(Inner_background , 70, 70); //270, 10
		// lv_obj_add_style(Inner_background,&Genesis_blue,LV_PART_INDICATOR); //lv-part-main
		// lv_obj_set_style_radius(Inner_background , LV_RADIUS_CIRCLE, 0);
		
		
		
		//LABELS
		lv_obj_t *leftDrivetrainLabel = lv_label_create(lv_scr_act());
		lv_label_set_text(leftDrivetrainLabel, "Left Drivetrain");
		lv_obj_set_pos(leftDrivetrainLabel, 175, 8); // x = 50, y = 50

		lv_obj_t *rightDrivetrainLabel = lv_label_create(lv_scr_act());
		lv_label_set_text(rightDrivetrainLabel, "Right Drivetrain");
		lv_obj_set_pos(rightDrivetrainLabel, 170, 43); // x = 50, y = 50

		lv_obj_t *intakeLabel = lv_label_create(lv_scr_act());
		lv_label_set_text(intakeLabel, "Intake");
		lv_obj_set_pos(intakeLabel, 195,80); // x = 50, y = 50

		lv_obj_t *armLabel = lv_label_create(lv_scr_act());
		lv_label_set_text(armLabel, "Arm");
		lv_obj_set_pos(armLabel, 200, 115); // x = 50, y = 50

		lv_obj_t *leftDrivetrainWatts = lv_label_create(lv_scr_act());
		lv_label_set_text(leftDrivetrainWatts, "Left Drive");
		lv_obj_set_pos(leftDrivetrainWatts, 137, 215); //130
		
		lv_obj_t *rightDrivetrainWatts = lv_label_create(lv_scr_act());
		lv_label_set_text(rightDrivetrainWatts, "Right Drive");
		lv_obj_set_pos(rightDrivetrainWatts, 225, 215); //220
		
		lv_obj_t *IntakeWatts = lv_label_create(lv_scr_act());
		lv_label_set_text(IntakeWatts, "Intake");
		lv_obj_set_pos(IntakeWatts, 329, 215); //325

		lv_obj_t *ArmWatts = lv_label_create(lv_scr_act());
		lv_label_set_text(ArmWatts, "Arm");
		lv_obj_set_pos(ArmWatts, 425, 215);
		pros::delay(100);
	}
}