void indoor_navigation_withWiFi_kf(Initialization_Pkt* ini, Meas meas, Sol_Pkt*  sol)
{
    static UINT8 fitst_time_flag = 0;
    static NavIndoor nav;
    static KF_4 kf_4;
    static MemoIndoor memo;


    static UINT8 if_heading_initialized = 0;
    static UINT8 if_position_initialized = 0;
    static UINT8 if_nav_initialized = 0;
    static UINT8 if_has_wifi_result_for_initialization = 0;
    static UINT8 if_has_changed_Qk = 0;

    //static FLOAT32 gyro_bias[3];
    //static FLOAT32 accel_bias[3];
    int i, j, k, i_mid_in_memo, i_heading_in_memo;
    double min_t_heading_in_memo[2];
    float max_min_qs_detect[2], diff_max_min_gz;
    float ma_mid1, min_t1;
    float max_va, min_va, t_max_va, t_min_va_4_step;
    int i_eph_in_memo_min_va;
    float heading;
    double t_mid1, lat1;
    static double ts_gyro = 0.0;
    double t_diff_g_a[LENGTH_MEMO_SENSOR_DATA];

    int if_detect_step, if_update_step, if_qs;
    int if_update_pos = 0;

    static double t0_gyro = 0.00;
    static double t_pre_gyro = 0.00;
    static int if_first_t_gyro = 0;

    static double t_pre_accel = 0.00;
    static double t0_accel = 0.00;
    static int if_first_t_accel = 0;

    static double t_pre_mag = 0.00;
    static double t0_mag = 0.00;
    static int if_first_t_mag = 0;
    static int if_first_t_mag2 = 0;
    static int i_count_mag_data = 0;

    static double t_pre_baro = 0.00;
    static double t0_baro = 0.00;
    static int if_first_t_baro = 0;
    static int if_first_t_baro2 = 0;
    static int i_count_baro_data = 0;

    //WiFi
    int availability_wifi[1] = {0};
    int num_floor[1] = {1};		// Temperory
    static double pos_wifi[3] = {0.00};   // The height should be set here because it is not dealed with in the wifi function


    // GPS result
    double pos_gps_result[3] = {0.00};
    float accuracy_gps_result = 0.0;
    static int if_gps_can_defeat_wifi;
    static double t_gps_defeat_start;
    int if_during_defeat_wifi_time;

    int if_gps_accu_can_cali_hei;
    static int eph_gps_calibrate_height;

    // MM result
    double pos_mm_result[3] = {0.00};


    //Map
    //static float da_map_NE[MAX_FLOOR_LINE_NUM][8];
    static float da_map_NE[N_ROW_DB_FP][N_COL_DB_FP];
    static int nLine_link[1];
    UINT8 if_align_heading;
    //FLOAT32 ang_align;
    int i_link_align_heading = 0;

    float WSH[MAX_FLOOR_LINE_NUM], WSPD[MAX_FLOOR_LINE_NUM], TWS[MAX_FLOOR_LINE_NUM];
    float rprp_NE[MAX_FLOOR_LINE_NUM][2], DD[MAX_FLOOR_LINE_NUM]; //,  diff_heading_step_heading;
    float r1_NE[2], r2_NE[2], rs_NE[2], rp_NE[2], delta_beta1, max_tws[2], rp_Link[2], dist_4_rej_link_pos;
    float d_North, d_East;

    // KF
    FLOAT32 PHI[N_STATES][N_STATES], PHIT[N_STATES][N_STATES], PHIP[N_STATES][N_STATES], PHIPPHIT[N_STATES][N_STATES];
    FLOAT32 Hk_2[2][N_STATES],Rk_2[2][2], Zk_2[2],inno_2[2], Hk_2xk[3];
    FLOAT32 xk[N_STATES] = {0.0};
    FLOAT32* pPHI[N_STATES], *pPHIT[N_STATES], *pPHIP[N_STATES], *pPHIPPHIT[N_STATES];
    FLOAT32* pHk_2[2], *pRk_2[2];
    static float pos_NEH[3], pos_NEH_meas[3];
    int if_kf_updated = 0;

    // -----------------  Attitude and Misalignment ----------------------
    static Att_Pkt att_pkt;
    static Misalign_pkt misa;

    DOUBLE64 t_cur;
    FLOAT32 f_b[3], w_b[3], m_b[3], fb_sm[3], w_b_zero[3];
    float acc_temp;
    UINT8 flag_accel, flag_mag;
    UINT8 flag_ini_heading;
    FLOAT32 ini_heading;  //Deg
    UINT8 seed_availability;
    float seed_bias[3];

    FLOAT32 f_leveled[3];
    float f_n[3], norm_a_n, m_n[3];
    FLOAT32 gyro_norm, rad_rotation;
    FLOAT32 R_ls[3][3];

    FLOAT32 memo_acc_norm[N_MEM_misO_1_STEP_MIS];    // Put into memo later
    UINT8 i_memo_acc_norm = 0;

    float mean_acc_norm;
    float da_Acc_Variance_Data;
    INT8 height_change_Data;	 // => Height changing       2 ===> Stairs Up    3 ===> Stairs Down

    FLOAT32 misalignment_uesd = 0.0;
    // ===============  End Attitude and Misalignment ====================



    // ----------------------------- Barometer --------------------------------
    static UINT8 if_has_baro = 0;
    static FLOAT32 t_pre_stair = -99.0;

    UINT8 if_has_baro_in_step;
    static UINT8 this_is_step = 0;

    static int if_first_floor_3_KCCI = 0;
    static int if_first_floor_2_KCCI = 0;
    static int if_first_floor_1_KCCI = 0;
    static int if_first_floor_1_PDC = 0;
    static int if_first_floor_2_PDC = 0;
    static int if_first_floor_3_PDC = 0;


    static int if_this_is_first_GPS = 0;

    static int if_not_in_KCCI = 0;

    static int if_this_is_first_gps_height = 0;

    static double t_start_PDC = 0.0;
    static double t_end_PDC_1 = 0.0;
    static double t_start_PDC2 = 0.0;
    static double t_end_PDC2_1 = 0.0;

    static int if_first_start_outdoor_gps = 0;
    static double t_start_outdoor_gps = 0.0;


    static int if_start_use_wifi = 0;
    double TH_START_USE_WIFI = 30.0;
    // ============================= Barometer ================================


    // ------ For debug only --------
    //static int i_step_debug;
    // ====== For debug only ========


    // --------------------------------------------------------------------
    // Set indicators in sol to "0"
    sol->avail_sol_pdr = 0;
    sol->avail_step = 0;
    sol->avail_heading = 0;
    sol->avail_heading_step = 0;
    sol->avail_sol_wifi = 0;
    sol->avail_att_mis = 0;
    sol->avail_has_baro_in_step = 0;
    sol->avail_heading_mag = 0;
    sol->avail_show_qs = 0;
    sol->avail_sol_mm_result_used = 0;
    sol->avail_sol_gps_result_used = 0;

    // ----------------------------------- Initialization --------------------------------------------
    if (!fitst_time_flag){
        fitst_time_flag = 1;

        sol->floor_num = 3;
        // for the first wifi data
        /*meas.t_gyro = 0.00;
         meas.t_accel = 0.00;
         meas.t_mag = 0.00;
         meas.t_baro = 0.00;*/

        //
        //gyro_bias[0] = ini_bg[0];
        //gyro_bias[1] = ini_bg[1];
        //gyro_bias[2] = ini_bg[2];   // Later we can save and read gBias from a file

        if_gps_can_defeat_wifi = 0;
        t_gps_defeat_start = 0.00;
        eph_gps_calibrate_height = 0;



        att_pkt.bg_estimated[0] = ini_gyro_bias_att[0];
        att_pkt.bg_estimated[1] = ini_gyro_bias_att[1];
        att_pkt.bg_estimated[2] = ini_gyro_bias_att[2];

        att_pkt.ba_estimated[0] = ini_accel_bias_att[0];
        att_pkt.ba_estimated[1] = ini_accel_bias_att[1];
        att_pkt.ba_estimated[2] = ini_accel_bias_att[2];   // Later we can save and read gBias from a file

        //P, Qk
        for (i=0; i<N_STATES; i++) {
            for (j=0; j<N_STATES; j++) {
                kf_4.P[i][j] = 0.0;
                kf_4.Qk[i][j] = 0.0;
            }
        }
        if (N_STATES == 4)
        {
            kf_4.P[0][0] = INI_POS_VAR * INI_POS_VAR;
            kf_4.P[1][1] = INI_POS_VAR * INI_POS_VAR;
            kf_4.P[2][2] = INI_HEAD_VAR * INI_HEAD_VAR;
            kf_4.P[3][3] = INI_SL_VAR * INI_SL_VAR;

            kf_4.Qk[0][0] = QK_POS_PHASE_1 * QK_POS_PHASE_1;
            kf_4.Qk[1][1] = QK_POS_PHASE_1 * QK_POS_PHASE_1;
            kf_4.Qk[2][2] = QK_HEAD_PHASE_1 * QK_HEAD_PHASE_1;
            kf_4.Qk[3][3] = QK_SL * QK_SL;
        }
        else if (N_STATES == 3)
        {
            kf_4.P[0][0] = INI_POS_VAR * INI_POS_VAR;
            kf_4.P[1][1] = INI_POS_VAR * INI_POS_VAR;
            kf_4.P[2][2] = INI_HEAD_VAR * INI_HEAD_VAR;

            kf_4.Qk[0][0] = QK_POS_PHASE_1 * QK_POS_PHASE_1;
            kf_4.Qk[1][1] = QK_POS_PHASE_1 * QK_POS_PHASE_1;
            kf_4.Qk[2][2] = QK_HEAD_PHASE_1 * QK_HEAD_PHASE_1;
        }
        ChangeArrayMemoryLayout_float(N_STATES,N_STATES,kf_4.pP,(FLOAT32 *)kf_4.P,0);         //P
        ChangeArrayMemoryLayout_float(N_STATES,N_STATES,kf_4.pQk,(FLOAT32 *)kf_4.Qk,0);       //Qk

        // misa
        misa.Misalign_Az = 0.0;
        if (MODE_MISALIGNMENT == 1)
        {
            misa.Misalign_Az = FIXED_MISALIGNEMNT;
        }

        // -- Initialization PDR
        nav.heading_platform = 0.0;

        //if (ini->type_initializaiton == 1)
        if(TYPE_INITIALIZATION != 100)
        {
            nav.pos[0] = INI_POS[0];
            nav.pos[1] = INI_POS[1];
            nav.pos[2] = INI_POS[2];

            //nav.heading_platform = INI_HEADING;
            nav.heading_platform = INI_HEADING;    // This is platform heading
        }


        nav.heading_step = nav.heading_platform;
        nav.t_step1 = 0;
        nav.sl = SL;

        nav.floor_num = INI_FLOOR_NUM;

        //-- End Initialization PDR

        // ---- INITIALIZAION_MEMO
        memo.i_accel = 0;
        memo.i_gyro = 0;
        memo.i_mag = 0;
        memo.i_baro = 0;

        memo.i_ma = 0;
        memo.i_t_step = 0;
        memo.i_pos_pdr = 0;
        memo.i_va = 0;
        memo.i_memo_heading_step = 0;
        memo.i_misa = 0;

        memo.i_memo_baro = 0;
        memo.i_memo_baro_sm = 0;
        memo.i_memo_baro_change = 0;
        memo.i_memo_if_stair = 0;
        memo.i_memo_status_stair = 0;
        memo.i_memo_floor_num = 0;

        memo.i_memo_norm_gyro_QS = 0;
        //memo.i_memo_if_vertical_gyro = 0;

        for(i=0; i<LENGTH_MEMO_SENSOR_DATA; i++){
            memo.t_accel[i] = 0.00;
            memo.a_x[i] = 0.0;
            memo.a_y[i] = 0.0;
            memo.a_z[i] = 0.0;
            memo.t_gyro[i] = 0.00;
            memo.g_x[i] = 0.0;
            memo.g_y[i] = 0.0;
            memo.g_z[i] = 0.0;

            memo.gz_level[i] = 0.0;
            memo.heading_platform[i] = 0.0;
        }

        // baro
        memo.height_pre = 1000.0;             // Needs to change
        for (i=0; i<N_EPH_SMOOTH_BARO; i++)
        {
            memo.t_baro[i] = 0.0;
            memo.baro[i] = 0.0;
        }
        for (i=0; i<N_EPH_BARO_DIFF; i++)
        {
            memo.baro_sm[i] = 0;
        }
        for (i=0; i<N_MEMO_BARO_CHANGE; i++)
        {
            memo.baro_change[i] = 0;
        }
        for (i=0; i<N_MEMO_IF_STAIR; i++)
        {
            memo.if_stair[i] = 0;
        }
        for (i=0; i<N_MEMO_STATUS_STAIR; i++)
        {
            memo.status_stair[i] = 0;
        }
        for (i=0; i<N_MEMO_FLOOR_NUM; i++)
        {
            memo.floor_num[i] = 1;
        }

        // Mag
        nav.declination = INI_DECLINATION;
        for (i=0; i< N_EPH_MAG_DATA; i++)
        {
            memo.t_mag[i] = 0.0;
            memo.m_x[i] = 0.0;
            memo.m_y[i] = 0.0;
            memo.m_z[i] = 0.0;
        }
        for (i=0; i<N_EPH_MEMO_NORM; i++)
        {
            memo.normm[i] = 0.0;
        }
        for (i=0; i<N_EPH_MEMO_IND_QSMF; i++)
        {
            memo.ind_qsmf[i] = 0;
        }
        for (i=0; i<N_EPH_HEADING_MAG; i++)
        {
            memo.heading_mag[i] = 0.0;
        }


        //
        for(i=0; i<LENGTH_MEMO_MA; i++){
            memo.ma[i] = 0.0;
        }
        for(i=0; i<LENGTH_MEMO_T_STEP; i++) {
            memo.t_step[i] = 0.00;
        }
        for(i=0; i<LENGTH_MEMO_POS_PDR; i++) {
            memo.t_pos_pdr[i] = 0.00;
            memo.pos_pdr_lat[i] = 0.00;
            memo.pos_pdr_lon[i] = 0.00;
            memo.pos_pdr_hei[i] = 0.00;
            memo.heading_step[i] = 0.0;
        }
        for(i=0; i<N_MEMO_VA; i++)
        {
            memo.va[i] = 0.0;
            memo.t_va[i] = 0.00;
        }
        for (i=0; i<N_SM_MISALIGN; i++)
        {
            memo.misalign[i] = 0.0;
        }



        // ---- End INITIALIZAION_MEMO


        /*memo.i_pos_pdr ++;
         memo.pos_pdr_lat[memo.i_pos_pdr-1] = nav.pos[0];
         memo.pos_pdr_lon[memo.i_pos_pdr-1] = nav.pos[1];
         memo.pos_pdr_hei[memo.i_pos_pdr-1] = nav.pos[2];


         sol->avail_sol_pdr = 1;
         sol->sol_t_sol_pdr = memo.t_pos_pdr[memo.i_pos_pdr-1];
         sol->sol_pdr[0] = memo.pos_pdr_lat[memo.i_pos_pdr-1]/D2R;
         sol->sol_pdr[1] = memo.pos_pdr_lon[memo.i_pos_pdr-1]/D2R;
         sol->sol_pdr[2] = memo.pos_pdr_hei[memo.i_pos_pdr-1];*/

        //fprintf(sol->f_out_rec_pos, "%.4f %.8f %.8f %.3f\n",
        //		memo.t_pos_pdr[memo.i_pos_pdr-1], memo.pos_pdr_lat[memo.i_pos_pdr-1]/D2R,
        //		memo.pos_pdr_lon[memo.i_pos_pdr-1]/D2R, memo.pos_pdr_hei[memo.i_pos_pdr-1]);


        // --------------------new for map
        if (IF_ALIGN_TO_LINKS || IF_PROJECT_TO_LINK)
        {
            //load_DB_fp_txt(f_DB_fp_txt, da_map_NE, nLine_link);
            load_DB_fp_string(da_map_NE, nLine_link);
        }
        // ---------------------
    }  // End (!fitst_time_flag)

    pos_NEH[0] = (nav.pos[0]-ZERO_POS[0])*RM;
    pos_NEH[1] = (nav.pos[1]-ZERO_POS[1])*RN_COS_LON;
    // ===================================  End of Initialization ====================================



    // -------------------- 2015-04-03 - First should be Accel ---------------------
    if (memo.i_accel == 0)
    {
        if (meas.avail_gyro == 1)
        {
            meas.avail_gyro = 0;
        }
        if (meas.avail_mag == 1)
        {
            meas.avail_mag = 0;
        }
        if (meas.avail_baro == 1)
        {
            meas.avail_baro = 0;
        }
    }
    // ==================== 2015-04-03 - First should be Accel =====================



    // Timestamp
    if (meas.avail_gyro)
    {
        if (if_first_t_gyro) {
            meas.t_gyro -= t0_gyro;  // t0_gyro is used for time_system for gyro
            if (fabs(meas.t_gyro-t_pre_gyro) < TH_REJECT_GYRO_OUTPUT) {  // t_pre_gyro is used for gyro time in the previous epoch
                meas.avail_gyro = 0;
            }
            else {
                t_pre_gyro = meas.t_gyro;
            }

        }
        else {
            if (!memo.i_accel) {			// The first one should be ACCEL!
                meas.avail_gyro = 0;
            }
            else {
                t0_gyro = meas.t_gyro;
                meas.t_gyro = 0;
                if_first_t_gyro = 1;

                t_pre_gyro = meas.t_gyro;
            }
        }
        //t_pre_gyro = meas.t_gyro // up
    }

    if (meas.avail_accel) {
        if (if_first_t_accel) {
            meas.t_accel -= t0_accel;
            if (fabs(meas.t_accel-t_pre_accel) < TH_REJECT_ACCEL_OUTPUT) {
                meas.avail_accel = 0;
            }
            else {
                t_pre_accel = meas.t_accel;
            }
        }
        else {
            t0_accel = meas.t_accel;
            meas.t_accel = 0;
            if_first_t_accel = 1;

            t_pre_accel = meas.t_accel;
        }
        //t_pre_accel = meas.t_accel;
    }

    if (meas.avail_mag)
    {
        i_count_mag_data ++;
        meas.avail_mag = 0;

        if (!if_first_t_mag)
        {
            if (!memo.i_accel) {
                meas.avail_mag = 0;
            }
            else {
                if_first_t_mag = 1;
                t0_mag = meas.t_mag;

                t_pre_mag = meas.t_mag;
            }
        }

        if (i_count_mag_data > I_START_MAG_DATA)
        {
            i_count_mag_data = I_START_MAG_DATA;

            meas.t_mag -= t0_mag;
            meas.avail_mag = 1;
            if (fabs(meas.t_mag-t_pre_mag) < TH_REJECT_MAG_OUTPUT) {  //
                meas.avail_mag = 0;
            }
            if (!if_first_t_mag2)
            {
                if_first_t_mag2 = 1;
                // First mag <====================================================
                t_pre_mag = meas.t_mag;
            }
            //t_pre_mag = meas.t_mag;
        }
    }

    if (meas.avail_baro)
    {
        i_count_baro_data ++;
        meas.da_baro = cal_height_baro(meas.da_baro);   // pressure -> height

        meas.avail_baro = 0;

        if (!if_first_t_baro)
        {
            if (!memo.i_accel) {
                meas.avail_baro = 0;
            }
            else {
                if_first_t_baro = 1;
                t0_baro = meas.t_baro;
            }
        }

        if (i_count_baro_data > I_START_BARO_DATA)
        {
            i_count_baro_data = I_START_BARO_DATA;

            meas.t_baro -= t0_baro;
            meas.avail_baro = 1;
            if (fabs(meas.t_baro-t_pre_baro) < TH_REJECT_BARO_OUTPUT) {  //
                meas.avail_baro = 0;
            }
            if (!if_first_t_baro2)
            {
                if_first_t_baro2 = 1;
                memo.height_baro0 = meas.da_baro;   // height_baro_0

                t_pre_baro = meas.t_baro;
            }
            //t_pre_baro = meas.t_baro;
        }
        nav.height_baro = meas.da_baro;
    }



    //// ------------------ IF CUT IMU -----------------------
    //if(IF_CUT_IMU) {
    //	if (meas.avail_gyro && meas.t_gyro < T_START_CIT_IMU) {
    //		meas.avail_gyro = 0;
    //	}
    //	if (meas.avail_accel && meas.t_accel < T_START_CIT_IMU) {
    //		meas.avail_accel = 0;
    //	}
    //	if (meas.avail_mag && meas.t_mag < T_START_CIT_IMU) {
    //		meas.avail_mag = 0;
    //	}
    //	if (meas.avail_baro && meas.t_baro < T_START_CIT_IMU) {
    //		meas.avail_baro = 0;
    //	}
    //}

    //// ================== IF CUT IMU =======================


    //pos_NEH[2] = nav.pos[2];
    // Start
    if (meas.avail_gyro)		// Put into attitude algorithm later
    {
        meas.da_gyro[0] = meas.da_gyro[0] * D2R ;  //gyro_bias[0];
        meas.da_gyro[1] = meas.da_gyro[1] * D2R ; //- gyro_bias[1];
        meas.da_gyro[2] = meas.da_gyro[2] * D2R ; //gyro_bias[2];

        if (!att_pkt.if_vertical)
        {
            w_b[0] = meas.da_gyro[0];
            w_b[1] = meas.da_gyro[1];
            w_b[2] = meas.da_gyro[2];
        }
        else if (att_pkt.if_vertical == 1)   // Vertical Axis
        {
            w_b[0] =  meas.da_gyro[2];
            w_b[1] =  meas.da_gyro[1];
            w_b[2] =  -meas.da_gyro[0];
        }
        else if (att_pkt.if_vertical == -1)
        {
            w_b[0] =  -meas.da_gyro[2];
            w_b[1] =  meas.da_gyro[1];
            w_b[2] =  meas.da_gyro[0];
        }

        compensate_b_att(w_b, att_pkt.bg_estimated);

        meas.da_gyro[0] = w_b[0];
        meas.da_gyro[1] = w_b[1];
        meas.da_gyro[2] = w_b[2];
    }

    if (meas.avail_accel) {
        if (!att_pkt.if_vertical)
        {
            f_b[0] = meas.da_accel[0];
            f_b[1] = meas.da_accel[1];
            f_b[2] = meas.da_accel[2];
        }
        else if (att_pkt.if_vertical == 1)   // Vertical Axis
        {
            f_b[0] =  meas.da_accel[2];
            f_b[1] =  meas.da_accel[1];
            f_b[2] =  -meas.da_accel[0];
        }
        else if (att_pkt.if_vertical == -1)
        {
            f_b[0] =  -meas.da_accel[2];
            f_b[1] =  meas.da_accel[1];
            f_b[2] =  meas.da_accel[0];
        }

        compensate_b_att(f_b, att_pkt.ba_estimated);

        meas.da_accel[0] = f_b[0];
        meas.da_accel[1] = f_b[1];
        meas.da_accel[2] = f_b[2];
    }

    if (meas.avail_mag) {
        if (!att_pkt.if_vertical)
        {
            m_b[0] = meas.da_mag[0];
            m_b[1] = meas.da_mag[1];
            m_b[2] = meas.da_mag[2];
        }
        else if (att_pkt.if_vertical == 1)   // Vertical Axis
        {
            m_b[0] =  meas.da_mag[2];
            m_b[1] =  meas.da_mag[1];
            m_b[2] =  -meas.da_mag[0];
        }
        else if (att_pkt.if_vertical == -1)
        {
            m_b[0] =  -meas.da_mag[2];
            m_b[1] =  meas.da_mag[1];
            m_b[2] =  meas.da_mag[0];
        }

        //compensate_b_att(m_b, att_pkt.ba_estimated);

        meas.da_mag[0] = m_b[0];
        meas.da_mag[1] = m_b[1];
        meas.da_mag[2] = m_b[2];
    }

    // ----------------------------------- Put into memory --------------------------------------------
    real_time_put_into_memo(&memo, meas, &att_pkt, &nav);

    // =================================== Put into memory ============================================




    // ----------------------------------   new for mag  ---------------------------------------------
    if (meas.avail_mag)
    {
        //Euler2Dcm_att_float_float_3_3_mis(att_pkt.att[0], att_pkt.att[1], 0, R_ls);
        //for(j = 0; j < 3; j++)
        //{
        //	m_n[j] = 0.0;
        //	for(k = 0; k < 3; k++)	// bringing in the accel into the level plane
        //	{
        //		m_n[j] += R_ls[j][k]*meas.da_mag[k];
        //	}
        //}

        //nav.declination = nav.declination;
        //nav.heading_mag = -atan2(m_n[1], m_n[0]) + nav.declination;   //+


        sol->avail_heading_mag = 1;
        sol->sol_t_heading_mag = meas.t_mag;
        sol->m_b[0] = meas.da_mag[0];
        sol->m_b[1] = meas.da_mag[1];
        sol->m_b[2] = meas.da_mag[2];
        sol->heading_mag = memo.heading_mag[memo.i_memo_heading_mag-1] / D2R; //nav.heading_mag / D2R;
        sol->declination = nav.declination / D2R;
        sol->indi_qsmf = memo.ind_qsmf[memo.i_memo_indi_qsmf-1];

    }
    // ==================================   new for mag  =============================================




    // ----------------------------------   new for baro  ---------------------------------------------
    if(meas.avail_baro && this_is_step)
    {
        this_is_step = 0;

        if_has_baro_in_step = 0;
        if (memo.i_t_step > 0 && fabs(memo.t_step[memo.i_t_step-1]-meas.t_baro)<TH_DIFF_BARO_STEP)
        {
            if_has_baro_in_step = 1;
        }

        if (if_has_baro_in_step)
        {
            memo.i_memo_baro ++;		  // -------------------------------Barometer LEVEL 1
            if (memo.i_memo_baro <= N_EPH_SMOOTH_BARO )
            {
                memo.baro[memo.i_memo_baro-1] = nav.height_baro;
            }
            else
            {
                memo.i_memo_baro = N_EPH_SMOOTH_BARO;
                for (k=0; k<N_EPH_SMOOTH_BARO-1; k++)
                {
                    memo.baro[k] = memo.baro[k+1];
                }
                memo.baro[N_EPH_SMOOTH_BARO-1] = nav.height_baro;
            }

            nav.height_baro_sm = cal_mean_float(memo.baro, memo.i_memo_baro);

            memo.i_memo_baro_sm ++;		   // -------------------------------Barometer LEVEL 2
            if (memo.i_memo_baro_sm <= N_EPH_BARO_DIFF)
            {
                memo.baro_sm[memo.i_memo_baro_sm-1] = nav.height_baro_sm;
            }
            else
            {
                memo.i_memo_baro_sm = N_EPH_BARO_DIFF;
                for (k=0; k<N_EPH_BARO_DIFF-1; k++)
                {
                    memo.baro_sm[k] = memo.baro_sm[k+1];
                }
                memo.baro_sm[N_EPH_BARO_DIFF-1] = nav.height_baro_sm;
            }

            nav.height_change = 0;
            if (memo.i_memo_baro_sm == N_EPH_BARO_DIFF)		// -------------------------------Barometer LEVEL 3
            {
                nav.height_change = memo.baro_sm[N_EPH_BARO_DIFF-1] - memo.baro_sm[0];

                memo.i_memo_baro_change++;
                if (memo.i_memo_baro_change <= N_MEMO_BARO_CHANGE)
                {
                    memo.baro_change[memo.i_memo_baro_change-1] = nav.height_change;
                }
                else
                {
                    memo.i_memo_baro_change = N_MEMO_BARO_CHANGE;
                    for (k=0; k<N_MEMO_BARO_CHANGE-1; k++)
                    {
                        memo.baro_change[k] = memo.baro_change[k+1];
                    }
                    memo.baro_change[N_MEMO_BARO_CHANGE-1] = nav.height_change;
                }
            }


//            // ----------------- NEW FOR COMPETITION -------------------------
//			if (fabs(nav.height_change) < TH_FLAT_FLOOR_CHANGE_HEIGHT0)
//			{
//				memo.height_baro0 += nav.height_change;
//				printf("%.2fs, height0 = %.2f, height_change = %.2f, height = %.2f\n", meas.t_baro, memo.height_baro0,nav.height_change, meas.da_baro);
//			}
//
//
//			// ------------------------- 4 DBs ------------------------- 20151007
//			float def_height_2_initial = sol->height_baro_sm - memo.height_baro0;
//
//
//			int HEIGHT_1_FLOOR = 3;     // ################################
//
//
//			if (def_height_2_initial > HEIGHT_1_FLOOR *3)
//				num_floor[0] = 4;
//			else if (def_height_2_initial > HEIGHT_1_FLOOR *2)
//				num_floor[0] = 3;
//			else if (def_height_2_initial > HEIGHT_1_FLOOR)
//				num_floor[0] = 2;
//			else
//				num_floor[0] = 1;
//
//			sol->floor_num = num_floor[0];
//			// ========================= 4 DBs =========================
//
//			// ================= NEW FOR COMPETITION =========================

            nav.flag_stair = 0;
            if (memo.i_memo_baro_change == N_MEMO_BARO_CHANGE )
            {
                if (memo.baro_change[N_MEMO_BARO_CHANGE-1] > TH_DETECT_STAIR &&
                    memo.baro_change[N_MEMO_BARO_CHANGE-2] > TH_DETECT_STAIR &&
                    memo.baro_change[N_MEMO_BARO_CHANGE-3] > TH_DETECT_STAIR &&
                    memo.baro_change[N_MEMO_BARO_CHANGE-4] > TH_DETECT_STAIR )
                {
                    nav.flag_stair = 1;
                }
                else if (memo.baro_change[N_MEMO_BARO_CHANGE-1] < -TH_DETECT_STAIR &&
                         memo.baro_change[N_MEMO_BARO_CHANGE-2] < -TH_DETECT_STAIR &&
                         memo.baro_change[N_MEMO_BARO_CHANGE-3] < -TH_DETECT_STAIR &&
                         memo.baro_change[N_MEMO_BARO_CHANGE-4] < -TH_DETECT_STAIR )
                {
                    nav.flag_stair = -1;
                }
            }

            memo.i_memo_if_stair++;	// -------------------------------Barometer LEVEL 4
            if (memo.i_memo_if_stair <= N_MEMO_IF_STAIR)
            {
                memo.if_stair[memo.i_memo_if_stair-1] = nav.flag_stair;
            }
            else
            {
                memo.i_memo_if_stair = N_MEMO_IF_STAIR;
                for (k=0; k<N_MEMO_IF_STAIR-1; k++)
                {
                    memo.if_stair[k] = memo.if_stair[k+1];
                }
                memo.if_stair[N_MEMO_IF_STAIR-1] = nav.flag_stair;
            }

            if (memo.i_memo_if_stair == N_MEMO_IF_STAIR)
            {
                nav.mean_if_stair = cal_mean_float((FLOAT32*)memo.if_stair, N_MEMO_IF_STAIR);
                if (nav.mean_if_stair > 0.5)
                {
                    nav.indi_status_stair = 1;
                }
                else if (nav.mean_if_stair < -0.5)
                {
                    nav.indi_status_stair = -1;
                }
                else
                {
                    nav.indi_status_stair = 0;
                }
            }
            memo.i_memo_status_stair++;	// -------------------------------Barometer LEVEL 5
            if (memo.i_memo_status_stair <= N_MEMO_STATUS_STAIR)
            {
                memo.status_stair[memo.i_memo_status_stair-1] = nav.indi_status_stair;
            }
            else
            {
                memo.i_memo_status_stair = N_MEMO_STATUS_STAIR;
                for (k=0; k<N_MEMO_STATUS_STAIR-1; k++)
                {
                    memo.status_stair[k] = memo.status_stair[k+1];
                }
                memo.status_stair[N_MEMO_STATUS_STAIR-1] = nav.indi_status_stair;
            }


            nav.if_change_floor = 0;


            nav.diff_height_to_height0 = nav.height_baro_sm - memo.height_baro0;
            // ----------------  Only for GNSS_CENTER -----------------
            if (nav.indi_status_stair == 0)
            {
                if (nav.diff_height_to_height0 < -2.5)
                {
                    if (nav.floor_num == 12)
                    {
                        nav.if_change_floor = 1;
                    }
                    nav.floor_num = 11;
                }
                else if (nav.diff_height_to_height0 > -1.5)
                {
                    if (nav.floor_num == 11)
                    {
                        nav.if_change_floor = 1;
                    }
                    nav.floor_num = 12;
                }
                else
                {
                    // ...
                }
            }
            // ================  Only for GNSS_CENTER ==================


            sol->avail_has_baro_in_step = 1;
            sol->sol_t_height_baro = meas.t_baro;
            sol->height_baro_sm = nav.height_baro_sm;
            sol->height_change = nav.height_change;
            sol->flag_stair = nav.flag_stair;
            sol->indi_status_stair = nav.indi_status_stair;
            //sol->floor_num = nav.floor_num;


            // ----------------- NEW FOR COMPETITION -------------------------
            if (fabs(nav.height_change) < TH_FLAT_FLOOR_CHANGE_HEIGHT0)
            {
                // memo.height_baro0 += nav.height_change;
                printf("%.2fs, height0 = %.2f, height_change = %.2f, height = %.2f\n", meas.t_baro, memo.height_baro0,nav.height_change, meas.da_baro);
            }


            // ------------------------- 4 DBs ------------------------- 20151007
            float def_height_2_initial = sol->height_baro_sm - memo.height_baro0;


            // int HEIGHT_1_FLOOR = 3;     // ################################

            num_floor[0] = 1;
            if (if_first_floor_3_KCCI == 0) {
                if_first_floor_3_KCCI = 1;
                memo.height_baro0 = sol->height_baro_sm;
            }
            if (if_first_floor_3_KCCI == 1 && if_first_floor_2_KCCI == 0 && if_first_floor_1_KCCI == 0 &&
                if_first_floor_1_PDC == 0 && if_first_floor_2_PDC == 0 && if_first_floor_3_PDC == 0)
            {
                num_floor[0] = 3;
                def_height_2_initial = sol->height_baro_sm - memo.height_baro0;
                if (def_height_2_initial < -2) {
                    if_first_floor_2_KCCI = 1;
                    memo.height_baro0 = memo.height_baro0 - 4;
                }
            }
            if (if_first_floor_3_KCCI == 1 && if_first_floor_2_KCCI == 1 && if_first_floor_1_KCCI == 0 &&
                if_first_floor_1_PDC == 0 && if_first_floor_2_PDC == 0 && if_first_floor_3_PDC == 0)
            {
                num_floor[0] = 2;
                def_height_2_initial = sol->height_baro_sm - memo.height_baro0;
                if (def_height_2_initial < -2) {
                    if_first_floor_1_KCCI = 1;
                    memo.height_baro0 = memo.height_baro0 - 4;
                }
            }

//			    if (if_this_is_first_GPS == 1 && meas.accuracy_gps_result < 7.1)
//				{
//					if_first_floor_1_PDC = 1;
//				}

            if (if_first_floor_3_KCCI == 1 && if_first_floor_2_KCCI == 1 && if_first_floor_1_KCCI == 1 &&
                if_first_floor_1_PDC == 1 && if_first_floor_2_PDC == 0 && if_first_floor_3_PDC == 0) {
                if (meas.accuracy_gps_result > 23.5 && meas.t_accel - t_start_outdoor_gps > 30.0)
                {
                    if (if_this_is_first_gps_height == 0)
                    {
                        if_this_is_first_gps_height = 1;
                        num_floor[0] = 1;
                        memo.height_baro0 = sol->height_baro_sm;

                        t_start_PDC = meas.t_accel;

                    }

                }
            }



            if (if_first_floor_3_KCCI == 1 && if_first_floor_2_KCCI == 1 && if_first_floor_1_KCCI == 1 && if_first_floor_1_PDC == 1 && if_first_floor_2_PDC == 0 && if_first_floor_3_PDC == 0) {
                def_height_2_initial = sol->height_baro_sm - memo.height_baro0;
                if (def_height_2_initial > 2.3) {
                    if_first_floor_2_PDC = 1;
                    memo.height_baro0 =  sol->height_baro_sm;

                    t_end_PDC_1 = meas.t_accel;
                    if(t_end_PDC_1 - t_start_PDC < 50)
                    {
                        // go back to 1
                        if_first_floor_2_PDC = 0;
                        num_floor[0] = 1;
                        memo.height_baro0 = sol->height_baro_sm;

                        t_start_PDC = t_end_PDC_1;
                    }

                    t_start_PDC2 = meas.t_accel;
                }
            }



            if (if_first_floor_3_KCCI == 1 && if_first_floor_2_KCCI == 1 && if_first_floor_1_KCCI == 1 && if_first_floor_1_PDC == 1 && if_first_floor_2_PDC == 1 && if_first_floor_3_PDC == 0) {
                def_height_2_initial = sol->height_baro_sm - memo.height_baro0;
                if (def_height_2_initial > 2) {
                    if_first_floor_3_PDC = 1;
                    memo.height_baro0 =  sol->height_baro_sm;
                    num_floor[0] = 3;

                    t_end_PDC2_1 = meas.t_accel;
                    if(t_end_PDC2_1 - t_start_PDC2 < 30)
                    {
                        // go back to 1
                        if_first_floor_3_PDC = 0;
                        num_floor[0] = 2;
                        memo.height_baro0 = sol->height_baro_sm;

                        t_start_PDC2 = t_end_PDC2_1;
                    }
                }
            }

            if (if_first_floor_3_KCCI == 1 && if_first_floor_2_KCCI == 0 && if_first_floor_1_KCCI == 0 && if_first_floor_1_PDC == 0 && if_first_floor_2_PDC == 0 && if_first_floor_3_PDC == 0) {
                num_floor[0] = 3;
            }
            else if (if_first_floor_3_KCCI == 1 && if_first_floor_2_KCCI == 1 && if_first_floor_1_KCCI == 0 && if_first_floor_1_PDC == 0 && if_first_floor_2_PDC == 0 && if_first_floor_3_PDC == 0) {
                num_floor[0] = 2;
            }
            else if (if_first_floor_3_KCCI == 1 && if_first_floor_2_KCCI == 1 && if_first_floor_1_KCCI == 1 && if_first_floor_1_PDC == 0 && if_first_floor_2_PDC == 0 && if_first_floor_3_PDC == 0) {
                num_floor[0] = 1;
            }
            else if (if_first_floor_3_KCCI == 1 && if_first_floor_2_KCCI == 1 && if_first_floor_1_KCCI == 1 && if_first_floor_1_PDC == 1 && if_first_floor_2_PDC == 0 && if_first_floor_3_PDC == 0) {
                num_floor[0] = 1;
            }
            else if (if_first_floor_3_KCCI == 1 && if_first_floor_2_KCCI == 1 && if_first_floor_1_KCCI == 1 && if_first_floor_1_PDC == 1 && if_first_floor_2_PDC == 1 && if_first_floor_3_PDC == 0) {
                num_floor[0] = 2;
            }
            else if (if_first_floor_3_KCCI == 1 && if_first_floor_2_KCCI == 1 && if_first_floor_1_KCCI == 1 && if_first_floor_1_PDC == 1 && if_first_floor_2_PDC == 1 && if_first_floor_3_PDC == 1) {
                num_floor[0] = 3;
            }

            sol->floor_num = num_floor[0];


            if (if_first_floor_1_PDC == 1)
            {
                if (if_not_in_KCCI)
                {
                    if_not_in_KCCI = 1;
                }
            }


//			float def_height_2_initial = sol->height_baro_sm - memo.height_baro0;
//
//
//			int HEIGHT_1_FLOOR = 3;     // ################################
//
//
//			if (def_height_2_initial > HEIGHT_1_FLOOR *3)
//				num_floor[0] = 4;
//			else if (def_height_2_initial > HEIGHT_1_FLOOR *2)
//				num_floor[0] = 3;
//			else if (def_height_2_initial > HEIGHT_1_FLOOR)
//				num_floor[0] = 2;
//			else
//				num_floor[0] = 1;
//
//			sol->floor_num = num_floor[0];
            // ========================= 4 DBs =========================

            // ================= NEW FOR COMPETITION =========================

        }  // End if (if_has_baro_in_step)
    }
    // =================================   new for baro  ================================================






    // --------------------------------- new for Initialization -------------------------------------
    if (!if_heading_initialized)
    {
        //if (ini->type_initializaiton == 100) {
        if(TYPE_INITIALIZATION == 100) {
            if (meas.avail_mag)
            {
                nav.heading_platform = memo.heading_mag[memo.i_memo_heading_mag-1];   //Heading to true north, rad
                if_heading_initialized = 1;
            }
        }
            //else if (ini->type_initializaiton == 0 || ini->type_initializaiton == 1) {
        else if (TYPE_INITIALIZATION == 0 || TYPE_INITIALIZATION == 1) {
            if_heading_initialized = 1;
        }
    }

    if (!if_position_initialized)
    {
        //if (ini->type_initializaiton == 100) {
        if(TYPE_INITIALIZATION == 100) {
            if (if_has_wifi_result_for_initialization)
            {
                nav.pos[0] = pos_wifi[0];
                nav.pos[1] = pos_wifi[1];
                //nav.pos[2] = ini->ini_pos[2];     // height to be determined by baro
                if_position_initialized = 1;
            }
        }
            //else if (ini->type_initializaiton == 0 || ini->type_initializaiton == 1) {
        else if (TYPE_INITIALIZATION == 0 || TYPE_INITIALIZATION == 1) {
            if_position_initialized = 1;
        }
    }

    if (!if_nav_initialized) {
        if (if_heading_initialized && if_position_initialized) {
            if_nav_initialized = 1;
        }
    }
    // ================================= new for Initialization =====================================







    // ----------------------------------- Heading Calculation ----------------------------------------

    if (if_nav_initialized && meas.avail_gyro)
    {
        if (memo.i_gyro>1)
        {
            ts_gyro = memo.t_gyro[memo.i_gyro-1]-memo.t_gyro[memo.i_gyro-2];
        }
        else {
            ts_gyro = 1.0/SENSORS_RATE_MOBILE;
        }

        if (memo.i_gyro < LENGTH_MEMO_SENSOR_DATA)
        {
            memo.heading_platform[memo.i_gyro-1] = nav.heading_platform;
        } else {

            for (i=0; i<LENGTH_MEMO_SENSOR_DATA-1; i++)
            {
                memo.heading_platform[i] = memo.heading_platform[i+1];
            }
            memo.heading_platform[LENGTH_MEMO_SENSOR_DATA-1] = nav.heading_platform;
        }


        // -------------------------------- Misalignment Estimator -------------------------------------
        if (memo.i_accel > 0)
        {
            f_b[0] = memo.a_x[memo.i_accel-1];
            f_b[1] = memo.a_y[memo.i_accel-1];
            f_b[2] = memo.a_z[memo.i_accel-1];

            // --------------------------------
            sol->avail_rp_accel = 1;
            sol->sol_t_rp_accel = meas.t_gyro;
            sol->sol_rp_accel[0] = cal_roll_att(f_b);
            sol->sol_rp_accel[1] = cal_pitch_att(f_b);
            sol->sol_rp_accel[0] /= D2R;
            sol->sol_rp_accel[1] /= D2R;
        }
        else
        {
            f_b[0] = 0.0;
            f_b[1] = 0.0;
            f_b[2] = -9.8;
        }

        if (IF_USE_MISALIGNMENT)
        {
            if (MODE_MISALIGNMENT == 0)
            {
                gyro_norm = sqrt(meas.da_gyro[0]*meas.da_gyro[0] + meas.da_gyro[1]*meas.da_gyro[1] + meas.da_gyro[2]*meas.da_gyro[2]);
                rad_rotation = 1 / D2R / gyro_norm;		 // Radius of Rotation

                Euler2Dcm_att_float_float_3_3_mis(att_pkt.att[0], att_pkt.att[1], 0, R_ls);
                for(j = 0; j < 3; j++)
                {
                    f_n[j] = 0.0;
                    for(k = 0; k < 3; k++)	// bringing in the accel into the level plane
                    {
                        //f_n[j] += R_ls[j][k]*fb_sm[k];
                        f_n[j] += R_ls[j][k]*f_b[k];
                    }

                    f_leveled[j] = f_n[j];
                }
                f_n[2] += Constant_Gravity;
                norm_a_n = f_n[0]*f_n[0] + f_n[1]*f_n[1] + f_n[2]*f_n[2];

                i_memo_acc_norm ++;
                if (i_memo_acc_norm <= N_MEM_misO_1_STEP_MIS)
                {
                    memo_acc_norm[i_memo_acc_norm-1] = norm_a_n;
                }
                else
                {
                    i_memo_acc_norm = N_MEM_misO_1_STEP_MIS;
                    for (k=0; k<N_MEM_misO_1_STEP_MIS-1; k++)
                    {
                        memo_acc_norm[k] = memo_acc_norm[k+1];
                    }
                    memo_acc_norm[N_MEM_misO_1_STEP_MIS-1] = norm_a_n;
                }

                mean_acc_norm = cal_mean_float_att(memo_acc_norm, i_memo_acc_norm);
                da_Acc_Variance_Data = cal_variance_float(memo_acc_norm,mean_acc_norm,i_memo_acc_norm);

                height_change_Data = 0;       // To be changed to the real detection of floor changing

                // Fill input structure for misalignment estimation
                misa.in_cur_Lev_Acc_Data[0] = f_leveled[0];  //Lev_Acc_Data[0][mm];
                misa.in_cur_Lev_Acc_Data[1] = f_leveled[1];  //Lev_Acc_Data[1][mm];
                misa.in_cur_Lev_Acc_Data[2] = f_leveled[2];  //Lev_Acc_Data[2][mm];
                misa.in_cur_pca_data_h = 0.0;
                misa.in_cur_pca_Acc_Mag = 0.0;
                // misa.in_cur_Lev_Acc_Window[3];
                misa.in_cur_RP_Data[0] = att_pkt.att[0];
                misa.in_cur_RP_Data[1] = att_pkt.att[1];
                misa.in_cur_Acc_Variance_Data = da_Acc_Variance_Data; //Acc_Variance_Data[mm];
                misa.in_cur_Rad_Rotation_Data = rad_rotation;  //Rad_Rotation_Data[mm];
                misa.in_cur_Gyr_Norm_Data = gyro_norm; // Gyr_Norm_Data[mm];
                misa.in_cur_Step_Frequency_Data = 0.0; // misa.Step_Frequency_Data[mm];
                misa.in_cur_vertical_motion_phase_Data = 0.0; //misa.vertical_motion_phase_Data[mm];
                misa.in_cur_motion_effective_coefficient_Data = 0.0; //misa.motion_effective_coefficient_Data[mm];
                misa.in_cur_Vertical_Change_Data = att_pkt.if_vertical;
                misa.in_cur_Height_Change_Data = height_change_Data;

                misalign_estimation_mis(&misa);

                if (IF_PLAY_WITH_MISALIGN)
                {
                    play_with_misalign(&misa, &att_pkt, &memo);
                    misalignment_uesd = misa.Misalign_Az_For_Use;
                }
                else
                {
                    misalignment_uesd = misa.Misalign_Az;
                }
            }
            else if (MODE_MISALIGNMENT == 1)
            {
                misa.Misalign_Az = FIXED_MISALIGNEMNT;
                misa.motion_mode = 0;
                misalignment_uesd = FIXED_MISALIGNEMNT;
            }

        }
        else   // if MODE_MISALIGNMENT != 0
        {
            misa.Misalign_Az = 0.0;
            misa.motion_mode = 0;
            misalignment_uesd = 0.0;
        }
        // ================================ Misalignment Estimator =====================================


        // ---------------------------------- QS -----------------------------------------
        //find_max_min_float_mis(memo.gz_level, max_min_qs_detect, memo.i_gyro);
        //diff_max_min_gz = max_min_qs_detect[0] - max_min_qs_detect[1];
        //if_qs = 0;
        //if (IF_USE_CURRENT_QS && diff_max_min_gz < TH_DETECT_QS_GYRO && memo.i_gyro == LENGTH_MEMO_SENSOR_DATA)

        find_max_min_float_mis(memo.norm_gyro, max_min_qs_detect, memo.i_gyro);
        diff_max_min_gz = max_min_qs_detect[0] - max_min_qs_detect[1];

        if_qs = 0;
        if (IF_USE_CURRENT_QS && diff_max_min_gz < TH_DETECT_QS_GYRO_NORM_20_DEG_S && memo.i_gyro == LENGTH_MEMO_SENSOR_DATA)
        {
            if_qs = 1;
        }
        sol->avail_show_qs = 1;
        sol->sol_t_qs = memo.t_gyro[memo.i_gyro-1];
        sol->if_qs = if_qs;
        sol->diff_norm_gyro = diff_max_min_gz / D2R;
        sol->norm_gyro = memo.norm_gyro[memo.i_gyro-1] / D2R;
        sol->i_memo_norm_gyro_QS = memo.i_memo_norm_gyro_QS;
        // ================================== QS =========================================


        if (!if_qs)
        {
            if (IF_USE_ATT_FILTER)
            {
                if (memo.i_accel > 0)
                {
                    flag_accel = 1;
                    fb_sm[0] = cal_mean_float(memo.a_x, memo.i_accel);
                    fb_sm[1] = cal_mean_float(memo.a_y, memo.i_accel);
                    fb_sm[2] = cal_mean_float(memo.a_z, memo.i_accel);

                    // --------------------------------
                    sol->sol_rp_accel_sm[0] = cal_roll_att(fb_sm);
                    sol->sol_rp_accel_sm[1] = cal_pitch_att(fb_sm);
                    sol->sol_rp_accel_sm[0] /= D2R;
                    sol->sol_rp_accel_sm[1] /= D2R;
                }
                else
                {
                    flag_accel = 0;
                    fb_sm[0] = 0.0;
                    fb_sm[1] = 0.0;
                    fb_sm[2] = -9.8;
                }


                // ------------------------ mag input -----------------------------
                flag_mag = 0;
                //float mean_ind_qsmf = cal_mean_float((FLOAT32*)memo.ind_qsmf,memo.i_memo_indi_qsmf);
                //if (IF_USE_MAG_UPDATE && memo.i_memo_normm == N_EPH_MEMO_NORM && fabs(memo.t_mag[memo.i_mag-1]-t_cur) < TH_TIME_DIFF_USE_ACC_MAG && mean_ind_qsmf > 0.5)
                //if (IF_USE_MAG_UPDATE && memo.i_memo_normm == N_EPH_MEMO_NORM && fabs(memo.t_mag[memo.i_mag-1]-t_cur) < TH_TIME_DIFF_USE_ACC_MAG)
                if (IF_USE_MAG_UPDATE && (!IF_USE_QSMF_LIMIT || (IF_USE_QSMF_LIMIT && memo.ind_qsmf[memo.i_memo_indi_qsmf-1])))
                {
                    flag_mag = 1;
                    m_b[0] = memo.m_x[memo.i_mag-1];
                    m_b[1] = memo.m_y[memo.i_mag-1];
                    m_b[2] = memo.m_z[memo.i_mag-1];

                    //if (!memo.ind_qsmf[memo.i_memo_indi_qsmf-1])     // The use of QSMF
                    //{
                    //	flag_mag = 0;
                    //}
                }
                else
                {
                    flag_mag = 0;
                    //m_b[0] = 0.0;
                    //m_b[1] = 0.0;
                    //m_b[2] = 0.0;
                }
                // ======================== mag input =============================







                // -------------------------------- Attitude Filter ------------------------------------



                //if (memo.i_gyro == 1)
                //{
                //	ini_heading = center_heading(nav.heading_platform + misa.Misalign_Az);  //Rad
                //}

                ini_heading = center_heading(nav.heading_platform + misa.Misalign_Az);  //Rad
                flag_ini_heading = 1;
                seed_availability = IF_SEED_INI_GYRO_BIAS_ATT;
                seed_bias[0] = ini_gyro_bias_att[0];
                seed_bias[1] = ini_gyro_bias_att[1];
                seed_bias[2] = ini_gyro_bias_att[2];

                attitude_filter_without_bg_conpensation_att(&att_pkt, &memo, meas.t_gyro, meas.da_gyro, fb_sm, flag_accel, m_b, flag_mag, ini_heading, flag_ini_heading, seed_bias, seed_availability);
                //attitude_filter_without_bg_conpensation_att(&att_pkt, t_cur, w_b, fb_sm, flag_accel, m_b, flag_mag, ini_heading, flag_ini_heading, seed_bias, seed_availability);
                //attitude_filter_without_bg_conpensation_att(&att_pkt, t_cur, w_b, f_b, flag_accel, m_b, flag_mag, ini_heading, flag_ini_heading, seed_bias, seed_availability);
                nav.heading_device = att_pkt.att[2];

                // ================================ End of Attitude Filter =====================================



                //att_pkt.heading_platform = center_heading(att_pkt.att[2] - misalignment_uesd);     // Device heading - misalignment = platform heading;
                nav.heading_platform = center_heading(att_pkt.att[2] - misalignment_uesd);     // Device heading - misalignment = platform heading; att_pkt.heading_platform;


                sol->avail_att_mis = 1;
                sol->sol_t_att_mis = att_pkt.t;
                sol->sol_att[0] = att_pkt.att[0] / D2R;
                sol->sol_att[1] = att_pkt.att[1] / D2R;
                sol->sol_att[2] = att_pkt.att[2] / D2R;
                sol->if_vertical = att_pkt.if_vertical;
                sol->sol_bg_estimated[0] = att_pkt.bg_estimated[0] / D2R;
                sol->sol_bg_estimated[1] = att_pkt.bg_estimated[1] / D2R;
                sol->sol_bg_estimated[2] = att_pkt.bg_estimated[2] / D2R;

                sol->misalign_az = misa.Misalign_Az / D2R;
                sol->motion_mode = misa.motion_mode;
                sol->fb_detect = misa.dir_s1;
                sol->step_frequency = misa.step_frequency_data;

                sol->misalignment_used = misalignment_uesd / D2R;
                sol->heading_platform = nav.heading_platform / D2R;
            }
            else   // if (!IF_USE_ATT_FILTER)
            {
                nav.heading_platform += memo.gz_level[memo.i_gyro-1]*ts_gyro;
                nav.heading_platform = center_heading(nav.heading_platform);

                sol->avail_att_mis = 0;
            }

        }
        else   // QS
        {
            if (IF_USE_ATT_FILTER)
            {

                flag_accel = 0;
                flag_mag = 0;
                w_b_zero[0] = 0.0;
                w_b_zero[1] = 0.0;
                w_b_zero[2] = 0.0;

                ini_heading = center_heading(nav.heading_platform + misa.Misalign_Az);  //Rad
                flag_ini_heading = 1;
                seed_availability = IF_SEED_INI_GYRO_BIAS_ATT;
                seed_bias[0] = ini_gyro_bias_att[0];
                seed_bias[1] = ini_gyro_bias_att[1];
                seed_bias[2] = ini_gyro_bias_att[2];

                attitude_filter_without_bg_conpensation_att(&att_pkt, &memo, meas.t_gyro, w_b_zero, fb_sm, flag_accel, m_b, flag_mag, ini_heading, flag_ini_heading, seed_bias, seed_availability);
                nav.heading_device = att_pkt.att[2];
                nav.heading_platform = center_heading(att_pkt.att[2] - misalignment_uesd);     // Device heading - misalignment = platform heading; att_pkt.heading_platform;

                sol->avail_att_mis = 1;
                sol->sol_t_att_mis = att_pkt.t;
                sol->sol_att[0] = att_pkt.att[0] / D2R;
                sol->sol_att[1] = att_pkt.att[1] / D2R;
                sol->sol_att[2] = att_pkt.att[2] / D2R;
                sol->if_vertical = att_pkt.if_vertical;
                sol->sol_bg_estimated[0] = att_pkt.bg_estimated[0] / D2R;
                sol->sol_bg_estimated[1] = att_pkt.bg_estimated[1] / D2R;
                sol->sol_bg_estimated[2] = att_pkt.bg_estimated[2] / D2R;

                sol->misalign_az = misa.Misalign_Az / D2R;
                sol->motion_mode = misa.motion_mode;
                sol->fb_detect = misa.dir_s1;
                sol->step_frequency = misa.step_frequency_data;

                sol->misalignment_used = misalignment_uesd / D2R;
                sol->heading_platform = nav.heading_platform / D2R;
            }
            else   // if (!IF_USE_ATT_FILTER)
            {
                nav.heading_platform += 0.0*ts_gyro;
                nav.heading_platform = center_heading(nav.heading_platform);

                sol->avail_att_mis = 0;
            }
        }

        //sol->count_rec_heading ++;
        sol->avail_heading = 1;
        sol->sol_t_heading = memo.t_gyro[memo.i_gyro-1];
        sol->sol_heading = memo.heading_platform[memo.i_gyro-1]/D2R;
        //fprintf(sol.f_out_rec_heading, "%.4f %.4f\n", memo.t_gyro[memo.i_gyro-1], memo.heading_platform[memo.i_gyro-1]);

    }
    // =================================== End of heading calculation ===================================




    // ----------------------------------- Step detection -----------------------------------------------
    // -- Step detection and PDR
    if (if_nav_initialized && meas.avail_accel)
    {
        if_detect_step = 0;
        if_update_step = 0;
        if (memo.i_va == N_MEMO_VA)
        {
            if_detect_step = 1;
        }

        if (if_detect_step)
        {
            /*max_va = -15.0;
             min_va = -5.0;*/
            max_va = 5.0;
            min_va = 15.0;
            t_max_va = 0.0;
            t_min_va_4_step = 0.0;
            i_eph_in_memo_min_va = -1;

            for(i=0; i<N_MEMO_VA; i++){
                if (memo.va[i] > max_va)
                {
                    max_va = memo.va[i];
                    t_max_va = memo.t_va[i];
                }
                if (memo.va[i] < min_va)
                {
                    min_va = memo.va[i];
                    t_min_va_4_step = memo.t_va[i];
                    i_eph_in_memo_min_va = i+1;						////// same as matlab
                }
            }

            if_update_step = 0;
            if (i_eph_in_memo_min_va == I_MIN_VA_IN_MEMO)
            {
                //if (max_va-min_va > TH_MAX_MIN_VA && min_va < TH_A_STEP2)
                if (min_va < TH_A_STEP2)
                    // if (max_va-min_va > TH_MAX_MIN_VA)
                {
                    if_update_step = 1;
                    if (memo.i_t_step)
                    {
                        if (t_min_va_4_step-memo.t_step[memo.i_t_step-1]<TH_MIN_T_DIFF_STEP)
                        {
                            if_update_step = 0;
                        }
                    }
                }
            }


            if (if_update_step)
            {
                //sol->count_rec_step ++;
                sol->avail_step = 1;
                sol->sol_t_step = t_min_va_4_step;
                sol->sol_step = min_va;
                //fprintf(sol.f_out_rec_step, "%.4f %.4f\n", t_mid1, ma_mid1);

                memo.i_t_step ++;
                if(memo.i_t_step <=LENGTH_MEMO_T_STEP){
                    memo.t_step[memo.i_t_step-1] = t_min_va_4_step;
                } else {
                    memo.i_t_step = LENGTH_MEMO_T_STEP;
                    for (i=0; i<LENGTH_MEMO_T_STEP-1; i++) {
                        memo.t_step[i] = memo.t_step[i+1];
                    }
                    memo.t_step[LENGTH_MEMO_T_STEP-1] = t_min_va_4_step;
                }

                nav.t_step0 = nav.t_step1;
                nav.t_step1 = t_min_va_4_step;
                nav.t_s_step = nav.t_step1 - nav.t_step0;

                if_update_pos = 1;
            }
        }
        // =================================== End of Step detection =====================================




        // ------------------------------------------- PDR -----------------------------------------------
        if (if_update_pos)			// Update PDR POS
        {
            this_is_step = 1;     // For using barometer

            //  ----------------------------------- Heading Adjust --------------------
            for (i=0; i<LENGTH_MEMO_SENSOR_DATA; i++){
                t_diff_g_a[i] = fabs(t_min_va_4_step - memo.t_gyro[i]);
            }
            min_double(t_diff_g_a, min_t_heading_in_memo, LENGTH_MEMO_SENSOR_DATA);
            i_heading_in_memo = floor(min_t_heading_in_memo[1]+0.001);
            nav.heading_step = memo.heading_platform[i_heading_in_memo];


            // -------------------------   new for map   -----------------------------
            if_align_heading = 0;
            if (IF_ALIGN_TO_LINKS && if_not_in_KCCI) {
                if_align_heading = map_adjusting(&nav, &att_pkt, &memo);
            }

            // =========================   new for map  =============================

            memo.i_memo_heading_step ++;
            if(memo.i_memo_heading_step <=LENGTH_MEMO_POS_PDR){
                memo.heading_step[memo.i_memo_heading_step-1] = nav.heading_step;
            } else {
                memo.i_memo_heading_step = LENGTH_MEMO_POS_PDR;
                for (i=0; i<LENGTH_MEMO_POS_PDR-1; i++) {
                    memo.heading_step[i] = memo.heading_step[i+1];
                }
                memo.heading_step[LENGTH_MEMO_POS_PDR-1] = nav.heading_step;
            }
            //  =================================== Heading Adjust ====================


            // pos_NEH
            d_North = nav.sl*cos(nav.heading_step);
            d_East  = nav.sl*sin(nav.heading_step);

            nav.pos[0] += d_North/RM;
            nav.pos[1] += d_East/RN_COS_LON;
            //nav.pos[2] = nav.pos[2];

            pos_NEH[0] += d_North;
            pos_NEH[1] += d_East;


            //pos_NEH[0] = (nav.pos[0]-ZERO_POS[0])*RM;
            //pos_NEH[1] = (nav.pos[1]-ZERO_POS[1])*RN_COS_LON;
            //pos_NEH[2] = nav.pos[2];

            // For debug
            sol->avail_heading_step = 1;
            sol->sol_t_heading_step = t_min_va_4_step;
            sol->sol_heading_step = nav.heading_step / D2R;


            // KF prediction
            ChangeArrayMemoryLayout_float(N_STATES,N_STATES,kf_4.pP,(FLOAT32 *)kf_4.P,0);         //P


            // ------- NEW - Change Qk values after a period -----------
            if (!if_has_changed_Qk && t_min_va_4_step > T_BETWEEN_PHASE_12)
            {
                if (N_STATES == 4)
                {
                    kf_4.Qk[0][0] = QK_POS_PHASE_2 * QK_POS_PHASE_2;
                    kf_4.Qk[1][1] = QK_POS_PHASE_2 * QK_POS_PHASE_2;
                    kf_4.Qk[2][2] = QK_HEAD_PHASE_2 * QK_HEAD_PHASE_2;
                    kf_4.Qk[3][3] = QK_SL * QK_SL;
                }
                else if (N_STATES == 3)
                {
                    kf_4.Qk[0][0] = QK_POS_PHASE_2 * QK_POS_PHASE_2;
                    kf_4.Qk[1][1] = QK_POS_PHASE_2 * QK_POS_PHASE_2;
                    kf_4.Qk[2][2] = QK_HEAD_PHASE_2 * QK_HEAD_PHASE_2;
                }
                if_has_changed_Qk = 1;
            }
            // ======= NEW - Change Qk values after a period ============

            ChangeArrayMemoryLayout_float(N_STATES,N_STATES,kf_4.pQk,(FLOAT32 *)kf_4.Qk,0);       //Qk

            if (N_STATES == 4)
            {
                PHI[0][0] = 1.0; PHI[0][1] = 0.0; PHI[0][2] = -nav.sl*sin(nav.heading_step); PHI[0][3] = cos(nav.heading_step);
                PHI[1][0] = 0.0; PHI[1][1] = 1.0; PHI[1][2] = nav.sl*cos(nav.heading_step);  PHI[1][3] = sin(nav.heading_step);
                PHI[2][0] = 0.0; PHI[2][1] = 0.0; PHI[2][2] = 1.0;  PHI[2][3] = 0.0;
                PHI[3][0] = 0.0; PHI[3][1] = 0.0; PHI[3][2] = 0.0;  PHI[3][3] = 1.0;
            }
            else if (N_STATES == 3)
            {
                PHI[0][0] = 1.0; PHI[0][1] = 0.0; PHI[0][2] = -nav.sl*sin(nav.heading_step);
                PHI[1][0] = 0.0; PHI[1][1] = 1.0; PHI[1][2] = nav.sl*cos(nav.heading_step);
                PHI[2][0] = 0.0; PHI[2][1] = 0.0; PHI[2][2] = 1.0;
            }

            ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pPHI,(FLOAT32 *)PHI,0);	              //Phi
            ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pPHIT,(FLOAT32 *)PHIT,0);			  //Phi'
            ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pPHIP,(FLOAT32 *)PHIP,0);			  //Phi*P
            ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pPHIPPHIT,(FLOAT32 *)PHIPPHIT,0);	  //Phi*P*Phi'

            //printf("\nP:\n");
            //printf("%.8f\t%.8f\t%.8f\t%.8f\n", kf_4.pP[0][0],kf_4.pP[0][1],kf_4.pP[0][2],kf_4.pP[0][3]);
            //printf("%.8f\t%.8f\t%.8f\t%.8f\n", kf_4.pP[1][0],kf_4.pP[1][1],kf_4.pP[1][2],kf_4.pP[1][3]);
            //printf("%.8f\t%.8f\t%.8f\t%.8f\n", kf_4.pP[2][0],kf_4.pP[2][1],kf_4.pP[2][2],kf_4.pP[2][3]);
            //printf("%.8f\t%.8f\t%.8f\t%.8f\n", kf_4.pP[3][0],kf_4.pP[3][1],kf_4.pP[3][2],kf_4.pP[3][3]);

            matrix_transpose_nxn_float(pPHI, N_STATES, N_STATES, pPHIT);						  //PHI'
            multiply_matrices_nxn_float(pPHI, kf_4.pP, N_STATES, N_STATES, N_STATES, pPHIP);	  //Phi*P
            multiply_matrices_nxn_float(pPHIP, pPHIT, N_STATES, N_STATES, N_STATES, pPHIPPHIT);	  //Phi*P*Phi'

            //printf("\nPHIPPHIT:\n");
            //printf("%.8f\t%.8f\t%.8f\t%.8f\n", pPHIPPHIT[0][0],pPHIPPHIT[0][1],pPHIPPHIT[0][2],pPHIPPHIT[0][3]);
            //printf("%.8f\t%.8f\t%.8f\t%.8f\n", pPHIPPHIT[1][0],pPHIPPHIT[1][1],pPHIPPHIT[1][2],pPHIPPHIT[1][3]);
            //printf("%.8f\t%.8f\t%.8f\t%.8f\n", pPHIPPHIT[2][0],pPHIPPHIT[2][1],pPHIPPHIT[2][2],pPHIPPHIT[2][3]);
            //printf("%.8f\t%.8f\t%.8f\t%.8f\n", pPHIPPHIT[3][0],pPHIPPHIT[3][1],pPHIPPHIT[3][2],pPHIPPHIT[3][3]);

            add_matrices_nxn(pPHIPPHIT, kf_4.pQk, N_STATES, N_STATES, kf_4.pP);					  //P = Phi*P*Phi'+ Qk

            //printf("\nP:\n");
            //printf("%.8f\t%.8f\t%.8f\t%.8f\n", kf_4.pP[0][0],kf_4.pP[0][1],kf_4.pP[0][2],kf_4.pP[0][3]);
            //printf("%.8f\t%.8f\t%.8f\t%.8f\n", kf_4.pP[1][0],kf_4.pP[1][1],kf_4.pP[1][2],kf_4.pP[1][3]);
            //printf("%.8f\t%.8f\t%.8f\t%.8f\n", kf_4.pP[2][0],kf_4.pP[2][1],kf_4.pP[2][2],kf_4.pP[2][3]);
            //printf("%.8f\t%.8f\t%.8f\t%.8f\n", kf_4.pP[3][0],kf_4.pP[3][1],kf_4.pP[3][2],kf_4.pP[3][3]);

            if(N_STATES == 4) {

            }
        } // end if_update_pos
    } // end if avail_accel
    // =================================== End of PDR ====================================================




    // ------------------------------------ KF Updates ---------------------------------------------------
    // ------------ Updates from Map matching ---------------
    if (if_nav_initialized && if_update_pos && IF_PROJECT_TO_LINK && if_align_heading)
    {
        rs_NE[0] = pos_NEH[0];
        rs_NE[1] = pos_NEH[1];

        for (j=0; j<nLine_link[0]; j++)
        {
            // Weight the proximity of the point to each link
            r1_NE[0] = da_map_NE[j][4];
            r1_NE[1] = da_map_NE[j][5];
            r2_NE[0] = da_map_NE[j][6];
            r2_NE[1] = da_map_NE[j][7];

            if (fabs(t_min_va_4_step - 43.7)<0.1)
            {
                t_min_va_4_step = t_min_va_4_step;
            }

            // DD[j] = ptoject2line(rs_NE, r1_NE, r2_NE, rp_NE);
            DD[j] = ptoject2line_can_be_outside(rs_NE, r1_NE, r2_NE, rp_NE);

            rprp_NE[j][0] = rp_NE[0];
            rprp_NE[j][1] = rp_NE[1];
            WSPD[j] = AP_4MAP/DD[j];

            // Weight the heading and bearing of the link
            delta_beta1 = fabs(dist_ang_rad(nav.heading_step,da_map_NE[j][3]));
            if (delta_beta1 > TH_MAX_DIFF_ANG_PROJECT_TO_LINK)
            {
                WSH[j] = WSH_NOT_SAME_ANG;
            }
            else {
                if (delta_beta1 < TH_MIN_DIFF_ANG_PROJECT_TO_LINK)
                {
                    delta_beta1 = AH_4MAP / TH_MIN_DIFF_ANG_PROJECT_TO_LINK;
                }
                WSH[j] = AH_4MAP / delta_beta1;
            }

            TWS[j] = WSPD[j] + WSH[j];
        }
        max_float(TWS, max_tws, nLine_link[0]);

        i_link_align_heading = (int)max_tws[1]+1;
        rp_Link[0] = rprp_NE[i_link_align_heading-1][0];
        rp_Link[1] = rprp_NE[i_link_align_heading-1][1];

        //printf("%-.3f %-.3f\n\n", rp_Link[0], rp_Link[1]);

        dist_4_rej_link_pos = cal_dist_2D_NE(rp_Link[0],rp_Link[1],pos_NEH[0],pos_NEH[1]);
        if (dist_4_rej_link_pos < TH_MAP_DIST_POINTS_REJECT && (i_link_align_heading == 1 || i_link_align_heading == 5))
            //if (dist_4_rej_link_pos < TH_MAP_DIST_POINTS_REJECT)
        {
            pos_NEH[0] = rp_Link[0];
            pos_NEH[1] = rp_Link[1];
            nav.pos[0] = pos_NEH[0] / RM + ZERO_POS[0];
            nav.pos[1] = pos_NEH[1] / RN_COS_LON + ZERO_POS[1];

            // DEBUG
            t_min_va_4_step = t_min_va_4_step;
            t_min_va_4_step = t_min_va_4_step;

        }
        i_link_align_heading = 0;
    }
    // ======== End of Updates from map matching ===========

    if_during_defeat_wifi_time = 0;

    if (IF_GPS_DEFEAT && IF_USE_GPS_RESULT) {
        if(IF_USE_WIFI && meas.avail_wifi){
            if (if_gps_can_defeat_wifi && meas.t_wifi - t_gps_defeat_start < TIME_DEFEAT_WIFI_LAST) {
                if_during_defeat_wifi_time = 1;
            }
        }
        if(IF_USE_MM_RESULT && meas.avail_mm_result){
            if (if_gps_can_defeat_wifi && meas.t_mm_result - t_gps_defeat_start < TIME_DEFEAT_WIFI_LAST) {
                if_during_defeat_wifi_time = 1;
            }
        }
    }

    // --------------- Updates from GPS result -------------------

    if(IF_USE_GPS_RESULT && meas.avail_gps_result && if_this_is_first_GPS == 0)
    {
        meas.avail_gps_result = 0;
        if_this_is_first_GPS = 1;
    }


    if (if_nav_initialized && IF_USE_GPS_RESULT && meas.avail_gps_result && meas.accuracy_gps_result <= TH_ACCUR_RELIABLE_GPS && meas.accuracy_gps_result > 0)
    {
        if (IF_GPS_DEFEAT && !if_during_defeat_wifi_time) {
            if (meas.accuracy_gps_result < TH_ACCUR_GPS_DEFEAT_WIFI) {
                if_gps_can_defeat_wifi = 1;
                t_gps_defeat_start = meas.t_gps_result;

                if (if_this_is_first_GPS == 1 && meas.accuracy_gps_result < 8.1)
                {
                    if_first_floor_1_PDC = 1;

                    if (if_first_start_outdoor_gps == 0)
                    {
                        if_first_start_outdoor_gps = 1;
                        t_start_outdoor_gps = meas.t_accel;
                    }
                }
            }
            else {
                if_gps_can_defeat_wifi = 0;
            }
        }

        if (IF_GPS_CALIBRATE_HEIGHT && IF_USE_GPS_RESULT) {
            if (meas.accuracy_gps_result > TH_ACCUR_GPS_CALIBRATE_HEIGHT) {
                eph_gps_calibrate_height = 0;
            }
            else
            {
                eph_gps_calibrate_height ++;
                if (eph_gps_calibrate_height > TH_EPH_GPS_CALI_HEIGHT) {
                    memo.height_baro0 = memo.height_pre;
                    eph_gps_calibrate_height = 0;
                }
            }
        }


        pos_gps_result[0] = meas.lat_gps_result * D2R;
        pos_gps_result[1] = meas.lon_gps_result * D2R;
        pos_gps_result[2] = meas.height_gps_result;
        accuracy_gps_result = meas.accuracy_gps_result;

        sol->avail_sol_gps_result_used = 1;
        sol->sol_t_sol_gps_result_used = meas.t_gps_result;
        sol->sol_gps_result_used[0] = pos_gps_result[0]/D2R;
        sol->sol_gps_result_used[1] = pos_gps_result[1]/D2R;
        sol->sol_gps_result_used[2] = pos_gps_result[2];
        sol->sol_gps_result_used[3] = accuracy_gps_result;

        // KF Update
        pos_NEH_meas[0] = (pos_gps_result[0]-ZERO_POS[0])*RM;
        pos_NEH_meas[1] = (pos_gps_result[1]-ZERO_POS[1])*RN_COS_LON;
        //pos_NEH_meas[2] = pos_wifi[2];

        ChangeArrayMemoryLayout_float(2,N_STATES,pHk_2,(FLOAT32 *)Hk_2,1);
        ChangeArrayMemoryLayout_float(2,2,pRk_2,(FLOAT32 *)Rk_2,0);		   //Rk = diag
        if (N_STATES == 4)
        {
            Hk_2[0][0] = 1; Hk_2[0][1] = 0; Hk_2[0][2] = 0; Hk_2[0][3] = 0;
            Hk_2[1][0] = 0; Hk_2[1][1] = 1; Hk_2[1][2] = 0; Hk_2[1][3] = 0;
        }
        else if (N_STATES == 3)
        {
            Hk_2[0][0] = 1; Hk_2[0][1] = 0; Hk_2[0][2] = 0;
            Hk_2[1][0] = 0; Hk_2[1][1] = 1; Hk_2[1][2] = 0;
        }

        Zk_2[0] = pos_NEH[0] - pos_NEH_meas[0];
        Zk_2[1] = pos_NEH[1] - pos_NEH_meas[1];

        multiply_matrix_with_vector_nxn(pHk_2, 2, N_STATES, xk, Hk_2xk);       //Hx = H*x
        float meas_noise_wifi_2[2] = {accuracy_gps_result, accuracy_gps_result};
        diag2_sq(meas_noise_wifi_2, 2, pRk_2);

        inno_2[0] = Zk_2[0] - Hk_2xk[0];
        inno_2[1] = Zk_2[1] - Hk_2xk[1];


        KF_update_indoor_float(2, xk, kf_4.pP, inno_2, pHk_2, pRk_2, N_STATES);

        if_kf_updated = 1;

        t_min_va_4_step = meas.t_accel;
    }
    // ======== End of Updates from GPS result ===========

    // --------------- Updates from WiFi -------------------
    if (IF_USE_WIFI && meas.avail_wifi)
    {
        if (meas.t_accel - t0_accel < TH_START_USE_WIFI)
        {
            meas.avail_wifi = 0;
        }
    }

    if (IF_USE_WIFI && meas.avail_wifi)
    {
        //num_floor[0] = 1;

        // ----- 6 DB
        if (if_first_floor_1_PDC == 0) {
            if (sol->floor_num == 3)
                num_floor[0] = 1;
            else if (sol->floor_num == 2)
                num_floor[0] = 2;
            else
                num_floor[0] = 3;
        }
        else if (if_first_floor_1_PDC == 1) {
            if (sol->floor_num == 1)
                num_floor[0] = 4;
            else if (sol->floor_num == 2)
                num_floor[0] = 5;
            else
                num_floor[0] = 6;
        }

        wifi_position_tracker_smooth(meas.name_ap, meas.rss_ap, meas.N_wifi, num_floor, pos_wifi, availability_wifi);

        if(availability_wifi[0]) {

            if (!if_has_wifi_result_for_initialization) {
                if_has_wifi_result_for_initialization = 1;
            }

            // Output WiFi
            sol->avail_sol_wifi = 1;
            sol->sol_t_sol_wifi = meas.t_wifi;
            sol->sol_wifi[0] = pos_wifi[0]/D2R;
            sol->sol_wifi[1] = pos_wifi[1]/D2R;
            sol->sol_wifi[2] = nav.pos[2];

            // KF Update
            pos_NEH_meas[0] = (pos_wifi[0]-ZERO_POS[0])*RM;
            pos_NEH_meas[1] = (pos_wifi[1]-ZERO_POS[1])*RN_COS_LON;
            //pos_NEH_meas[2] = pos_wifi[2];

            ChangeArrayMemoryLayout_float(2,N_STATES,pHk_2,(FLOAT32 *)Hk_2,1);
            ChangeArrayMemoryLayout_float(2,2,pRk_2,(FLOAT32 *)Rk_2,0);		   //Rk = diag
            if (N_STATES == 4)
            {
                Hk_2[0][0] = 1; Hk_2[0][1] = 0; Hk_2[0][2] = 0; Hk_2[0][3] = 0;
                Hk_2[1][0] = 0; Hk_2[1][1] = 1; Hk_2[1][2] = 0; Hk_2[1][3] = 0;
            }
            else if (N_STATES == 3)
            {
                Hk_2[0][0] = 1; Hk_2[0][1] = 0; Hk_2[0][2] = 0;
                Hk_2[1][0] = 0; Hk_2[1][1] = 1; Hk_2[1][2] = 0;
            }

            Zk_2[0] = pos_NEH[0] - pos_NEH_meas[0];
            Zk_2[1] = pos_NEH[1] - pos_NEH_meas[1];

            multiply_matrix_with_vector_nxn(pHk_2, 2, N_STATES, xk, Hk_2xk);       //Hx = H*x
            float meas_noise_wifi_2[2];
            if ((!IF_USE_GPS_RESULT) || (!IF_GPS_DEFEAT)) {
                meas_noise_wifi_2[0] = SIGMA_WIFI;
                meas_noise_wifi_2[1] = SIGMA_WIFI;
            }
            else {
                if (!if_during_defeat_wifi_time) {
                    meas_noise_wifi_2[0] = SIGMA_WIFI;
                    meas_noise_wifi_2[1] = SIGMA_WIFI;
                }
                else {
                    meas_noise_wifi_2[0] = SIGMA_WIFI_DEFEATED;
                    meas_noise_wifi_2[1] = SIGMA_WIFI_DEFEATED;
                }
            }
            diag2_sq(meas_noise_wifi_2, 2, pRk_2);

            inno_2[0] = Zk_2[0] - Hk_2xk[0];
            inno_2[1] = Zk_2[1] - Hk_2xk[1];

            KF_update_indoor_float(2, xk, kf_4.pP, inno_2, pHk_2, pRk_2, N_STATES);

            if_kf_updated = 1;
        }
    }
    // ======== End of Updates from WiFi ===========


    // --------------- Updates from MM result -------------------
    if (if_nav_initialized && IF_USE_MM_RESULT && meas.avail_mm_result)
    {
        pos_mm_result[0] = meas.lat_mm_result * D2R;
        pos_mm_result[1] = meas.lon_mm_result * D2R;
        pos_mm_result[2] = nav.pos[2];

        sol->avail_sol_mm_result_used = 1;
        sol->sol_t_sol_mm_result_used = meas.t_mm_result;
        sol->sol_mm_result_used[0] = pos_mm_result[0]/D2R;
        sol->sol_mm_result_used[1] = pos_mm_result[1]/D2R;
        sol->sol_mm_result_used[2] = pos_mm_result[2];

        // KF Update
        pos_NEH_meas[0] = (pos_mm_result[0]-ZERO_POS[0])*RM;
        pos_NEH_meas[1] = (pos_mm_result[1]-ZERO_POS[1])*RN_COS_LON;
        //pos_NEH_meas[2] = pos_wifi[2];

        ChangeArrayMemoryLayout_float(2,N_STATES,pHk_2,(FLOAT32 *)Hk_2,1);
        ChangeArrayMemoryLayout_float(2,2,pRk_2,(FLOAT32 *)Rk_2,0);		   //Rk = diag
        if (N_STATES == 4)
        {
            Hk_2[0][0] = 1; Hk_2[0][1] = 0; Hk_2[0][2] = 0; Hk_2[0][3] = 0;
            Hk_2[1][0] = 0; Hk_2[1][1] = 1; Hk_2[1][2] = 0; Hk_2[1][3] = 0;
        }
        else if (N_STATES == 3)
        {
            Hk_2[0][0] = 1; Hk_2[0][1] = 0; Hk_2[0][2] = 0;
            Hk_2[1][0] = 0; Hk_2[1][1] = 1; Hk_2[1][2] = 0;
        }

        Zk_2[0] = pos_NEH[0] - pos_NEH_meas[0];
        Zk_2[1] = pos_NEH[1] - pos_NEH_meas[1];

        multiply_matrix_with_vector_nxn(pHk_2, 2, N_STATES, xk, Hk_2xk);       //Hx = H*x
        float meas_noise_wifi_2[2];

        if ((!IF_USE_GPS_RESULT) || (!IF_GPS_DEFEAT)) {
            meas_noise_wifi_2[0] = SIGMA_MM_RESULT;
            meas_noise_wifi_2[1] = SIGMA_MM_RESULT;
        }
        else {
            if (!if_during_defeat_wifi_time) {
                meas_noise_wifi_2[0] = SIGMA_MM_RESULT;
                meas_noise_wifi_2[1] = SIGMA_MM_RESULT;
            }
            else {
                meas_noise_wifi_2[0] = SIGMA_MM_RESULT_DEFEATED;
                meas_noise_wifi_2[1] = SIGMA_MM_RESULT_DEFEATED;
            }
        }

        diag2_sq(meas_noise_wifi_2, 2, pRk_2);

        inno_2[0] = Zk_2[0] - Hk_2xk[0];
        inno_2[1] = Zk_2[1] - Hk_2xk[1];


        // ----- New AEKF
        if (IF_USE_AEKF_MM == 0)
        {
            KF_update_indoor_float(2, xk, kf_4.pP, inno_2, pHk_2, pRk_2, N_STATES);
        }
        else if (IF_USE_AEKF_MM == 1)
        {
            AEKF_MM_2_update_indoor_float(2, xk, kf_4.pP, inno_2, pHk_2, pRk_2, N_STATES);
        }

        if_kf_updated = 1;

    }
    // ======== End of Updates from MM result ===========
    // =================================== End of KF updates ==============================================



    // ------------------------------------ KF feedback ---------------------------------------------------
    if (if_nav_initialized && if_kf_updated)
    {
        if (N_STATES == 4)
        {
            pos_NEH[0] -= xk[0];
            pos_NEH[1] -= xk[1];
            nav.heading_step -= xk[2];

            // ------------------------------- NEW ON HEADING FEEDBACK !!!!!!!!!!!! - -----------------------
            nav.heading_platform -= xk[2];
            att_pkt.att[2] = center_heading(att_pkt.att[2] - xk[2]);
            Euler2Dcm_att(att_pkt.att, att_pkt.Cbn);
            Dcm2Quat_att(att_pkt.Cbn, att_pkt.qbn);
            // =============================== NEW ON HEADING FEEDBACK !!!!!!!!!!!! =========================


            nav.sl -= xk[3];

            xk[0] = 0.0;
            xk[1] = 0.0;
            xk[2] = 0.0;
            xk[3] = 0.0;

            nav.pos[0] = pos_NEH[0] / RM + ZERO_POS[0];
            nav.pos[1] = pos_NEH[1] / RN_COS_LON + ZERO_POS[1];

            if (meas.avail_gps_result && meas.accuracy_gps_result <= TH_ACCUR_VERY_RELIABLE_GPS) {    // VERY RELIABLE GPS!
                nav.pos[0] = meas.lat_gps_result * D2R;
                nav.pos[1] = meas.lon_gps_result * D2R;
            }
        }
        else if (N_STATES == 3)
        {

            pos_NEH[0] -= xk[0];
            pos_NEH[1] -= xk[1];
            nav.heading_step -= xk[2];

            // ------------------------------- NEW ON HEADING FEEDBACK !!!!!!!!!!!! - -----------------------
            nav.heading_platform -= xk[2];
            //nav.heading_device -= xk[2];
            att_pkt.att[2] = center_heading(att_pkt.att[2] - xk[2]);
            Euler2Dcm_att(att_pkt.att, att_pkt.Cbn);
            Dcm2Quat_att(att_pkt.Cbn, att_pkt.qbn);
            // =============================== NEW ON HEADING FEEDBACK !!!!!!!!!!!! =========================

            xk[0] = 0.0;
            xk[1] = 0.0;
            xk[2] = 0.0;

            nav.pos[0] = pos_NEH[0] / RM + ZERO_POS[0];
            nav.pos[1] = pos_NEH[1] / RN_COS_LON + ZERO_POS[1];

            if (IF_USE_GPS_RESULT == 1 && meas.avail_gps_result && meas.accuracy_gps_result <= TH_ACCUR_VERY_RELIABLE_GPS) {    // VERY RELIABLE GPS!
                nav.pos[0] = meas.lat_gps_result * D2R;
                nav.pos[1] = meas.lon_gps_result * D2R;
            }
        }

    }


    if (if_update_pos)
    {
        memo.i_pos_pdr ++;
        if(memo.i_pos_pdr <= LENGTH_MEMO_POS_PDR){
            memo.t_pos_pdr[memo.i_pos_pdr-1] = t_min_va_4_step;
            memo.pos_pdr_lat[memo.i_pos_pdr-1] = nav.pos[0];
            memo.pos_pdr_lon[memo.i_pos_pdr-1] = nav.pos[1];
            memo.pos_pdr_hei[memo.i_pos_pdr-1] = nav.pos[2];
        } else {
            memo.i_pos_pdr = LENGTH_MEMO_POS_PDR;
            for (i=0; i<LENGTH_MEMO_POS_PDR-1; i++) {
                memo.t_pos_pdr[i] = memo.t_pos_pdr[i+1];
                memo.pos_pdr_lat[i] = memo.pos_pdr_lat[i+1];
                memo.pos_pdr_lon[i] = memo.pos_pdr_lon[i+1];
                memo.pos_pdr_hei[i] = memo.pos_pdr_hei[i+1];
            }
            memo.t_pos_pdr[LENGTH_MEMO_POS_PDR-1] = t_min_va_4_step;
            memo.pos_pdr_lat[LENGTH_MEMO_POS_PDR-1] = nav.pos[0];
            memo.pos_pdr_lon[LENGTH_MEMO_POS_PDR-1] = nav.pos[1];
            memo.pos_pdr_hei[LENGTH_MEMO_POS_PDR-1] = nav.pos[2];
        }


        // POS!
        //sol.count_rec_pos ++;
        sol->avail_sol_pdr = 1;
        sol->sol_t_sol_pdr = memo.t_pos_pdr[memo.i_pos_pdr-1];
        sol->sol_pdr[0] = nav.pos[0]/D2R;
        sol->sol_pdr[1] = nav.pos[1]/D2R;
        sol->sol_pdr[2] = nav.pos[2];


        if (IF_USE_GPS_RESULT == 1 && meas.avail_gps_result && meas.accuracy_gps_result <= TH_ACCUR_VERY_RELIABLE_GPS) {    // VERY RELIABLE GPS!
            sol->sol_pdr[0] = meas.lat_gps_result;
            sol->sol_pdr[1] = meas.lon_gps_result;
        }

        /*sol->sol_pdr[0] = memo.pos_pdr_lat[memo.i_pos_pdr-1]/D2R;
         sol->sol_pdr[1] = memo.pos_pdr_lon[memo.i_pos_pdr-1]/D2R;
         sol->sol_pdr[2] = memo.pos_pdr_hei[memo.i_pos_pdr-1];*/

        //fprintf(sol.f_out_rec_pos, "%.4f %.8f %.8f %.3f\n",
        //		memo.t_pos_pdr[memo.i_pos_pdr-1], memo.pos_pdr_lat[memo.i_pos_pdr-1]/D2R,
        //		memo.pos_pdr_lon[memo.i_pos_pdr-1]/D2R, memo.pos_pdr_hei[memo.i_pos_pdr-1]);

    }

    // Set as "0"
    //meas.avail_gyro = 0;
    //meas.avail_accel = 0;
    // =================================== End of KF feedback ==============================================

    sol->height0 = memo.height_baro0;
}