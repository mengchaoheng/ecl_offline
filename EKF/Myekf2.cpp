#include "Myekf2.h"
#include <sstream>
#include <fstream>
#include <time.h>
#include <stdio.h>
#include<iostream>
#define ECL_STANDALONE

std::string num2str(float x)
{
    std::stringstream flowstream;
    flowstream<<x;
    return flowstream.str();
}

std::string mat2Str(float x[],int length){
    std::string str="";
    str.append(num2str(x[0]));
    int f;
    f=(int)(sizeof(x)/sizeof(float));

    for(int i=1;i<length;i++)
    {
        str.append(",");
        str.append(num2str(x[i]));
    }
    return str;
}


class Ekf2;


bool bReadGPS=false, bmagread=false, bReadBaro=false;//是否读取gps,mag,baro


namespace ekf2
{
Ekf2 *instance = nullptr;
}

Ekf2::Ekf2():
	_ekf(),
	_params(_ekf.getParamHandle())
{

}

Ekf2::~Ekf2()
{

}


void Ekf2::print_status()
{
	printf("local position OK %s", (_ekf.local_position_is_valid()) ? "[YES]" : "[NO]");
    printf("global position OK %s\n", (_ekf.global_position_is_valid()) ? "[YES]" : "[NO]");
}

void Ekf2::task_main()
{
    // std::ifstream imuread("../data/test/imu.txt");
    // std::ifstream gpsread("../data/test/gps.txt");
    // std::ifstream magread("../data/test/mag.txt");
    // std::ifstream airread("../data/test/baro.txt");
    std::ifstream imuread("../data/test/imu_data.txt");
    std::ifstream gpsread("../data/test/gps_data.txt");
    std::ifstream magread("../data/test/mag_data.txt");
    std::ifstream airread("../data/test/baro_data.txt");

    std::ofstream euler_estimator("../results/euler_estimator.txt");

    std::ofstream sensor_bias_out("../results/sensor_bias.txt");
    std::ofstream estimate_status_out("../results/estimate_status.txt");
    std::ofstream ekf_innovations_out("../results/ekf_innovations.txt");

    std::ofstream position_estimator("../results/position_estimator.txt");
    std::ofstream velocity_estimator("../results/velocity_estimator.txt");
    std::ofstream velocity_m("../results/velocity_m.txt");


	// initialise parameter cache// TODO
	//updateParams();
//	std::ifstream imuread("data/imu_data.txt");

    //vehicle_status_s vehicle_status = {};//增加一些飞机的状态



    uint64_t gyro_integral_dt = 0;
    float gyro_rad[3],accelerometer_m_s2[3];
	uint64_t accelerometer_integral_dt = 0;
	uint64_t last_IMUtime = 0;
    uint64_t now = 0;
    uint64_t mag_time_us_read=0;
    float magx, magy, magz;

    //baro data
    uint64_t baro_time_us_read=0;
    float baro_alt_meter, baro_temp_celcius, baro_pressure_pa, rho;
    //gps data
    uint64_t gps_time_us_read=0;
    uint64_t time_utc_usec=0;
    int64_t lat=0,lon=0,alt=0;
    int64_t alt_ellipsoid=0;
    float s_variance_m_s=0.f, c_variance_rad=0.f, eph=0.f, epv=0.f, hdop=0.f, vdop=0.f;
    int64_t noise_per_ms=0, jamming_indicator=0;
    float vel_m_s,vel_n_m_s, vel_e_m_s, vel_d_m_s,cog_rad;
    int64_t timestamp_time_relative;

    int64_t fix_type=0;
    bool vel_ned_valid;
    int64_t satellites_used=0;
    
    



    while (!_task_should_exit && !imuread.eof() && !gpsread.eof() && !magread.eof() && !airread.eof()) {

		bool isa = true;
		bool mag_updated = false;
		bool baro_updated = false;
		bool gps_updated = false;
        bool evq_updated=false;
        bool evp_updated=false;

		bool vehicle_status_updated = false;

		// long gyro_integral_dt = 0.01;
        imuread >> now;	//us

        // ECL_INFO("time now: %llu\n", now);

		// // push imu data into estimator
        float gyro_integral[3];
        imuread >> gyro_rad[0];	imuread >> gyro_rad[1];	imuread >> gyro_rad[2];imuread>>gyro_integral_dt;
        float gyro_dt = gyro_integral_dt / 1.e6f;
        // gyro_integral_dt /= 1.e6f;	//s
        ECL_INFO("[gyro]:now %llu, g1 %f, g2 %f, g3 %f, dt %llu s. \n",now, gyro_rad[0], gyro_rad[1], gyro_rad[2],gyro_integral_dt);

		gyro_integral[0] = gyro_rad[0] * gyro_dt;
		gyro_integral[1] = gyro_rad[1] * gyro_dt;
		gyro_integral[2] = gyro_rad[2] * gyro_dt;
        int64_t accelerometer_timestamp_relative;imuread>>accelerometer_timestamp_relative;
        float accel_integral[3];
        imuread >> accelerometer_m_s2[0];	imuread >> accelerometer_m_s2[1];	imuread >> accelerometer_m_s2[2];imuread>>accelerometer_integral_dt;
        // accelerometer_integral_dt/=1.e6;  //s
        float accel_dt = accelerometer_integral_dt / 1.e6f;
        // ECL_INFO("[acc]:now %llu, a1 %f, a2 %f, a3 %f, dt %llu s. \n",now,  accelerometer_m_s2[0], accelerometer_m_s2[1], accelerometer_m_s2[2],accelerometer_integral_dt);
		accel_integral[0] = accelerometer_m_s2[0] * accel_dt;
		accel_integral[1] = accelerometer_m_s2[1] * accel_dt;
		accel_integral[2] = accelerometer_m_s2[2] * accel_dt;
		_ekf.setIMUData(now, gyro_integral_dt, accelerometer_integral_dt, gyro_integral, accel_integral);		
		last_IMUtime = now;

        if(bmagread)
		{

            magread >> mag_time_us_read;	//us
            magread >> magx;
            magread >> magy;
            magread >> magz;
			//magx /= 100.0f;magy /= 100.0f;magz /= 100.0f;
            bmagread = false;

		}
        //if(mag_time_ms_read * 1.e3f < now)
        if(mag_time_us_read < now)
		{
			mag_updated = true;
            bmagread = true;
		}
		if(mag_updated)
		{
            //_timestamp_mag_us = mag_time_ms_read * 1.e3f;
            //_timestamp_mag_us = mag_time_ms_read;
            // _timestamp_mag_us = mag_time_us_read;
			// If the time last used by the EKF is less than specified, then accumulate the
			// data and push the average when the 50msec is reached.
			_mag_time_sum_ms += mag_time_us_read / 1000.0f;
			_mag_sample_count++;
			_mag_data_sum[0] += magx;
			_mag_data_sum[1] += magy;
			_mag_data_sum[2] += magz;
			uint32_t mag_time_ms = _mag_time_sum_ms / _mag_sample_count;
			
			if (mag_time_ms - _mag_time_ms_last_used > _params->sensor_interval_min_ms) {
				float mag_sample_count_inv = 1.0f / (float)_mag_sample_count;
                // calculate mean of measurements and correct for learned bias offsets
				float mag_data_avg_ga[3] = {_mag_data_sum[0] *mag_sample_count_inv, _mag_data_sum[1] *mag_sample_count_inv, _mag_data_sum[2] *mag_sample_count_inv};

				_ekf.setMagData(1000 * (uint64_t)mag_time_ms, mag_data_avg_ga);
                // ECL_INFO("[mag]: now %llu time %llu time_agv %u m1 %f m2 %f m3 %f\n",now,mag_time_us_read,mag_time_ms, _mag_data_sum[0],_mag_data_sum[1],_mag_data_sum[2]);
				_mag_time_ms_last_used = mag_time_ms;
				_mag_time_sum_ms = 0;
				_mag_sample_count = 0;
				_mag_data_sum[0] = 0.0f;
				_mag_data_sum[1] = 0.0f;
				_mag_data_sum[2] = 0.0f;	
			}		
		}


		if(bReadBaro)
		{
            airread >> baro_time_us_read;	//us

            airread>>baro_alt_meter;
            airread>>baro_temp_celcius;airread>>baro_pressure_pa;//baro_temp_celcius,baro_pressure_pa

            airread >>rho;
			// if(baroHeight_origin == 0)
			// 	baroHeight_origin = baroHeight;
			// baroHeight -= baroHeight_origin;
            bReadBaro= false;

		}
        if(baro_time_us_read <now)
		{
			baro_updated = true;
			bReadBaro = true;
		}
		if(baro_updated)
		{
                // _timestamp_balt_us = baro_time_us_read;

				// If the time last used by the EKF is less than specified, then accumulate the
				// data and push the average when the 50msec is reached.
				_balt_time_sum_ms += baro_time_us_read / 1000;
				_balt_sample_count++;
                _balt_data_sum += baro_alt_meter;
				uint32_t balt_time_ms = _balt_time_sum_ms / _balt_sample_count;

				if (balt_time_ms - _balt_time_ms_last_used > (uint32_t)_params->sensor_interval_min_ms) {
					float balt_data_avg = _balt_data_sum / (float)_balt_sample_count;
                // ECL_INFO("[baro]: now %llu time %llu time_vag %u balt_data_avg %f\n",now,baro_time_us_read,balt_time_ms, balt_data_avg);
                _ekf.set_air_density(rho);


                //from v1.8.2 i dont why do this
                // calculate static pressure error = Pmeas - Ptruth
                // model position error sensitivity as a body fixed ellipse with different scale in the positive and negtive X direction
                float _aspd_max=20.0f,_K_pstatic_coef_xp=0.0f,_K_pstatic_coef_xn=0.0f,_K_pstatic_coef_y=0.0f,_K_pstatic_coef_z=0.0f;
                const float max_airspeed_sq = _aspd_max * _aspd_max;
                float K_pstatic_coef_x;

                const Vector3f vel_body_wind = get_vel_body_wind();

                if (vel_body_wind(0) >= 0.0f) {
                    K_pstatic_coef_x = _K_pstatic_coef_xp;

                } else {
                    K_pstatic_coef_x = _K_pstatic_coef_xn;
                }

                const float x_v2 = fminf(vel_body_wind(0) * vel_body_wind(0), max_airspeed_sq);
                const float y_v2 = fminf(vel_body_wind(1) * vel_body_wind(1), max_airspeed_sq);
                const float z_v2 = fminf(vel_body_wind(2) * vel_body_wind(2), max_airspeed_sq);

                const float pstatic_err = 0.5f * rho *
                              (K_pstatic_coef_x * x_v2) + (_K_pstatic_coef_y * y_v2) + (_K_pstatic_coef_z * z_v2);

                // correct baro measurement using pressure error estimate and assuming sea level gravity
                balt_data_avg += pstatic_err / (rho * CONSTANTS_ONE_G);

                _ekf.setBaroData(1000 * (uint64_t)balt_time_ms, balt_data_avg);
                _balt_time_ms_last_used = balt_time_ms;
                _balt_time_sum_ms = 0;
                _balt_sample_count = 0;
                _balt_data_sum = 0.0f;

            }
		}

		if(bReadGPS)
		{

            // gpsread >> gps_time_us_read;	//us
            // gpsread>>tempgps;//time_utc_usec
            // gpsread>>lat;gpsread>>lon;gpsread>>alt;
            // gpsread>>tempgps;//alt_ellipsoid
            // gpsread>>pos_itow_ms;
            // gpsread>>tempgps;/*fix_quality*/gpsread>>tempgps;gpsread>>sacc;gpsread>>tempgps;/*c_var_rad*/
            // gpsread>>eph;gpsread>>epv;gpsread>>tempgps;/*hdop*/gpsread>>tempgps;/*vdop*/gpsread>>tempgps;/*noise_per_ms*/
            // gpsread>>tempgps;/*jamming_indicator*/gpsread>>vel_m_s;/*vel*/gpsread>>vel_n_m_s;gpsread>>vel_e_m_s;gpsread>>vel_d_m_s;
            // gpsread>>cog_rad;gpsread>>tempgps;/*timerealtive*/gpsread>>fix_type;gpsread>>vel_valid;gpsread>>nsats;

            // float temp; gpsread >> temp;
            gpsread >> gps_time_us_read;	//us
            gpsread >>time_utc_usec;
			gpsread >> lat;
			gpsread >> lon;
			gpsread >> alt;
			gpsread>>alt_ellipsoid;
			gpsread>>s_variance_m_s;
            gpsread>>c_variance_rad;
            gpsread>>eph;
            gpsread>>epv;
			gpsread>>hdop;
            gpsread>>vdop;
            gpsread>>noise_per_ms;
            gpsread>>jamming_indicator;
			gpsread >> vel_m_s;
			gpsread>>vel_n_m_s;
            gpsread>>vel_e_m_s;
            gpsread>>vel_d_m_s;
			gpsread>>cog_rad;
            gpsread>>timestamp_time_relative;
			gpsread>>fix_type;
            gpsread>>vel_ned_valid;
			gpsread>>satellites_used;
            ECL_INFO("[gps]: gps_time_us_read %llu, time_utc_usec %llu, lat %lld, lon %lld, alt %lld, alt_ellipsoid %lld, s_variance_m_s %f, c_variance_rad %f, eph %f, epv %f, hdop %f, vdop %f, noise_per_ms %lld, jamming_indicator %lld, vel_m_s %f, v1 %f, v2 %f, v3 %f, cog_rad %f, timestamp_time_relative %lld, fix_type %lld, vel_ned_valid %d, satellites_used %lld\n", gps_time_us_read, time_utc_usec, lat, lon, alt, alt_ellipsoid, s_variance_m_s, c_variance_rad, eph, epv, hdop, vdop, noise_per_ms, jamming_indicator, vel_m_s, vel_n_m_s, vel_e_m_s, vel_d_m_s, cog_rad, timestamp_time_relative, fix_type, vel_ned_valid, satellites_used);
            // ECL_INFO("[gps]: now %llu time %llu\n", now, gps_time_us_read);
            bReadGPS = false;

		}
        if(gps_time_us_read  < now)
		{
			gps_updated = true;
			bReadGPS = true;
		}
		if(gps_updated)
		{
			struct gps_message gps_msg = {};
            gps_msg.time_usec = gps_time_us_read;
            gps_msg.lat = lat;
            gps_msg.lon = lon;
            gps_msg.alt = alt;
            ECL_DEBUG("time now: %ld\n", now);

            gps_msg.fix_type = fix_type;
            gps_msg.eph = eph;
            gps_msg.epv = epv;
            gps_msg.sacc = s_variance_m_s;
            gps_msg.vel_m_s = vel_m_s;
            gps_msg.vel_ned[0] = vel_n_m_s;
            gps_msg.vel_ned[1] = vel_e_m_s;
            gps_msg.vel_ned[2] = vel_d_m_s;
            gps_msg.vel_ned_valid = vel_ned_valid;
            gps_msg.nsats = satellites_used;
            // ECL_INFO("gps_time_us_read:%lf,velocity: %f,%f,%f\n", gps_time_us_read,vel_n_m_s,vel_e_m_s,vel_d_m_s);
// ECL_INFO("[gps]: now %d time %d v1 %f v2 %f v3 %f\n",now,gps_time_us_read,vel_n_m_s,vel_e_m_s,vel_d_m_s);
            // printf("[gps]: time %llu, %d, %d, %d, %d\n",gps_msg.time_usec,gps_msg.lat,gps_msg.lon,gps_msg.alt,gps_msg.fix_type);
            // printf("[gps]: time %lf, %f, %f, %f, %f\n",gps_msg.time_usec,gps_msg.vel_n_m_s,gps_msg.vel_e_m_s,gps_msg.vel_d_m_s,gps_msg.vel_m_s);
            // ECL_INFO("now:%llu,velocity: %f,%f,%f\n", gps_msg.time_usec,gps_msg.vel_ned[0],gps_msg.vel_ned[1],gps_msg.vel_ned[2]);
			//TODO add gdop to gps topic
			gps_msg.gdop = 0.0f;

			_ekf.setGpsData(gps_msg.time_usec, &gps_msg);
		}


		//run the EKF update and output
		if (_ekf.update()) {

// ECL_INFO("time now: %llu\n", now);
            // integrate time to monitor time slippage
            if (_start_time_us == 0) {
                _start_time_us = now;
                _last_time_slip_us = 0;

            } else if (_start_time_us > 0) {
                _integrated_time_us += gyro_integral_dt;
                _last_time_slip_us = (now - _start_time_us) - _integrated_time_us;
            }


            //output vehicle_attitude_s data
			matrix::Quaternion<float> q;
			_ekf.copy_quaternion(q.data());
            // In-run bias estimates
            float gyro_bias[3];
            std::string space_str=",";
            _ekf.get_gyro_bias(gyro_bias);
            {
                // generate vehicle attitude quaternion data
                vehicle_attitude_s att;
                att.timestamp = now;

                q.copyTo(att.q);
                _ekf.get_quat_reset(&att.delta_q_reset[0], &att.quat_reset_counter);

                att.rollspeed = gyro_rad[0] - gyro_bias[0];
                att.pitchspeed = gyro_rad[1] - gyro_bias[1];
                att.yawspeed = gyro_rad[2] - gyro_bias[2];

                // publish vehicle attitude
                //std::stringstream attitude_strs;

                //std::string file;
                // euler_estimator<<att.timestamp<<space_str<<att.rollspeed<<space_str<<att.pitchspeed<<space_str<<att.yawspeed<<space_str<<att.q[0]<<space_str<<att.q[1]<<space_str<<att.q[2]<<space_str<<att.q[3]<<space_str<<att.delta_q_reset[0]<<space_str<<att.delta_q_reset[1]<<space_str<<att.delta_q_reset[2]<<space_str<<att.delta_q_reset[3]<<space_str<<att.quat_reset_counter<<std::endl;
                //printf((attitude_strs.str()));

                //euler_estimator<<attitude_strs.str()<<std::endl;

            }

            // generate vehicle local position data
            vehicle_local_position_s lpos={};
            lpos.timestamp = now;

            // Position of body origin in local NED frame
            float position[3];
            _ekf.get_position(position);

            // Local Position NED
            //printf("position: %lf,%lf,%lf\n", position[0], position[1], position[2]);
            position_estimator<< now <<" "<<position[0] <<" "<<position[1] <<" "
            <<-position[2] <<" "<<std::endl;


            const float lpos_x_prev = lpos.x;
            const float lpos_y_prev = lpos.y;
            lpos.x = (_ekf.local_position_is_valid()) ? position[0] : 0.0f;
            lpos.y = (_ekf.local_position_is_valid()) ? position[1] : 0.0f;
            lpos.z = position[2];

            // Velocity of body origin in local NED frame (m/s)
            float velocity[3];
            _ekf.get_velocity(velocity);

            velocity_estimator<< now <<" "<<velocity[0] <<" "<<velocity[1] <<" "
            <<-velocity[2] <<" "<<std::endl;
            velocity_m<< now <<" "<<vel_n_m_s <<" "<<vel_e_m_s <<" "
            <<-vel_d_m_s <<" "<<std::endl;


            lpos.vx = velocity[0];
            lpos.vy = velocity[1];
            lpos.vz = velocity[2];

            // vertical position time derivative (m/s)
            _ekf.get_pos_d_deriv(&lpos.z_deriv);

            // Acceleration of body origin in local NED frame
            float vel_deriv[3];
            _ekf.get_vel_deriv_ned(vel_deriv);
            lpos.ax = vel_deriv[0];
            lpos.ay = vel_deriv[1];
            lpos.az = vel_deriv[2];

            // TODO: better status reporting
            lpos.xy_valid = _ekf.local_position_is_valid() && !_preflt_horiz_fail;
            lpos.z_valid = !_preflt_vert_fail;
            lpos.v_xy_valid = _ekf.local_position_is_valid() && !_preflt_horiz_fail;
            lpos.v_z_valid = !_preflt_vert_fail;

            // Position of local NED origin in GPS / WGS84 frame
            map_projection_reference_s ekf_origin;
            uint64_t origin_time;
            // true if position (x,y,z) has a valid WGS-84 global reference (ref_lat, ref_lon, alt)
            const bool ekf_origin_valid = _ekf.get_ekf_origin(&origin_time, &ekf_origin, &lpos.ref_alt);
            lpos.xy_global = ekf_origin_valid;
            lpos.z_global = ekf_origin_valid;

            if (ekf_origin_valid && (origin_time > lpos.ref_timestamp)) {
                lpos.ref_timestamp = origin_time;
                lpos.ref_lat = ekf_origin.lat_rad * 180.0 / M_PI; // Reference point latitude in degrees
                lpos.ref_lon = ekf_origin.lon_rad * 180.0 / M_PI; // Reference point longitude in degrees
            }

            // The rotation of the tangent plane vs. geographical north
            matrix::Eulerf euler(q);

            euler_estimator<< now <<" "<<euler.phi() <<" "<<euler.theta() <<" "
				<<euler.psi() <<" "<<std::endl;	


            lpos.yaw = euler.psi();

            lpos.dist_bottom_valid = _ekf.get_terrain_valid();

            float terrain_vpos;
            _ekf.get_terrain_vert_pos(&terrain_vpos);
            lpos.dist_bottom = terrain_vpos - lpos.z; // Distance to bottom surface (ground) in meters

            // constrain the distance to ground to _rng_gnd_clearance
            if (lpos.dist_bottom < _rng_gnd_clearance) {
                lpos.dist_bottom = _rng_gnd_clearance;
            }

            lpos.dist_bottom_rate = -lpos.vz; // Distance to bottom surface (ground) change rate

            _ekf.get_ekf_lpos_accuracy(&lpos.eph, &lpos.epv);
            _ekf.get_ekf_vel_accuracy(&lpos.evh, &lpos.evv);

            // get state reset information of position and velocity
            _ekf.get_posD_reset(&lpos.delta_z, &lpos.z_reset_counter);
            _ekf.get_velD_reset(&lpos.delta_vz, &lpos.vz_reset_counter);
            _ekf.get_posNE_reset(&lpos.delta_xy[0], &lpos.xy_reset_counter);
            _ekf.get_velNE_reset(&lpos.delta_vxy[0], &lpos.vxy_reset_counter);

            // get control limit information
            _ekf.get_ekf_ctrl_limits(&lpos.vxy_max, &lpos.vz_max, &lpos.hagl_min, &lpos.hagl_max);

            // convert NaN to INFINITY
            if (!PX4_ISFINITE(lpos.vxy_max)) {
                lpos.vxy_max = INFINITY;
            }

            if (!PX4_ISFINITE(lpos.vz_max)) {
                lpos.vz_max = INFINITY;
            }

            if (!PX4_ISFINITE(lpos.hagl_min)) {
                lpos.hagl_min = INFINITY;
            }

            if (!PX4_ISFINITE(lpos.hagl_max)) {
                lpos.hagl_max = INFINITY;
            }
  

            //vehicle_local_out<<lposStream.str()<<std::endl;
            // ECL_INFO("now:%ld,velocity: %f,%f,%f\n", now,velocity[0], velocity[1], velocity[2]);
            // ECL_INFO("now:%ld,position: %lf,%lf,%lf\n",now, position[0], position[1], position[2]);



            {
                // publish all corrected sensor readings and bias estimates after mag calibration is updated above
                float accel_bias[3];
                _ekf.get_accel_bias(accel_bias);

                sensor_bias_s bias;

                bias.timestamp = now;

                bias.accel_x = accelerometer_m_s2[0] - accel_bias[0];
                bias.accel_y = accelerometer_m_s2[1] - accel_bias[1];
                bias.accel_z = accelerometer_m_s2[2] - accel_bias[2];

                bias.gyro_x_bias = gyro_bias[0];
                bias.gyro_y_bias = gyro_bias[1];
                bias.gyro_z_bias = gyro_bias[2];

                bias.accel_x_bias = accel_bias[0];
                bias.accel_y_bias = accel_bias[1];
                bias.accel_z_bias = accel_bias[2];

                bias.mag_x_bias = _last_valid_mag_cal[0];
                bias.mag_y_bias = _last_valid_mag_cal[1];
                bias.mag_z_bias = _last_valid_mag_cal[2];

//                if (_sensor_bias_pub == nullptr) {
//                    _sensor_bias_pub = orb_advertise(ORB_ID(sensor_bias), &bias);

//                } else {
//                    orb_publish(ORB_ID(sensor_bias), _sensor_bias_pub, &bias);
//                }
                //发布传感器改正数据
                //std::stringstream sensorBiasStream;
                sensor_bias_out<<bias.timestamp<<space_str<<bias.accel_x<<space_str<<bias.accel_y<<space_str<<bias.accel_z
                               <<bias.gyro_x_bias<<space_str<<bias.gyro_y_bias<<space_str<<bias.gyro_z_bias<<space_str<<bias.accel_x_bias
                              <<space_str<<bias.accel_y_bias<<space_str<<bias.accel_z_bias<<space_str<<bias.mag_x_bias<<space_str<<bias.mag_y_bias
                             <<space_str<<bias.mag_z_bias<<std::endl;
                //sensor_bias_out<<sensorBiasStream.str()<<std::endl;
                // ECL_INFO("[sensor_bias]now:%ld,time:%ld,gyro:%f %f %f",now,bias.timestamp,bias.accel_x_bias,bias.accel_y,bias.accel_z);


            }

            // publish estimator status
            estimator_status_s status;
            status.timestamp = now;
            _ekf.get_state_delayed(status.states);
            _ekf.get_covariances(status.covariances);
            _ekf.get_gps_check_status(&status.gps_check_fail_flags);
            _ekf.get_control_mode(&status.control_mode_flags);
            _ekf.get_filter_fault_status(&status.filter_fault_flags);
            _ekf.get_innovation_test_status(&status.innovation_check_flags, &status.mag_test_ratio,
                            &status.vel_test_ratio, &status.pos_test_ratio,
                            &status.hgt_test_ratio, &status.tas_test_ratio,
                            &status.hagl_test_ratio, &status.beta_test_ratio);

            status.pos_horiz_accuracy =lpos.eph;
            status.pos_vert_accuracy = lpos.epv
                    ;
            _ekf.get_ekf_soln_status(&status.solution_status_flags);
            _ekf.get_imu_vibe_metrics(status.vibe);
            status.time_slip = _last_time_slip_us / 1e6f;
            status.nan_flags = 0; // unused
            status.health_flags = 0; // unused
            status.timeout_flags = 0; // unused
            status.pre_flt_fail = _preflt_fail;

//            if (_estimator_status_pub == nullptr) {
//                _estimator_status_pub = orb_advertise(ORB_ID(estimator_status), &status);

//            } else {
//                orb_publish(ORB_ID(estimator_status), _estimator_status_pub, &status);
//            }
            //发布EKF滤波信息（主要是滤波约束条件）
            std::stringstream ekfStatusStream;
            std::string states_str=mat2Str(status.states,sizeof(status.states)/sizeof(float));//状态向量
            std::string cov_str=mat2Str(status.covariances,sizeof(status.covariances)/sizeof(float));//状态向量协方差
            estimate_status_out<<status.timestamp<<space_str<<states_str<<space_str<<status.n_states<<space_str<<status.vibe[0]
                          <<space_str<<status.vibe[1]<<space_str<<status.vibe[2]<<space_str<<cov_str<<space_str<<status.control_mode_flags<<space_str
                         <<status.pos_horiz_accuracy<<space_str<<status.pos_vert_accuracy<<space_str<<status.mag_test_ratio<<space_str<<status.vel_test_ratio
                        <<space_str<<status.pos_test_ratio<<space_str<<status.hgt_test_ratio<<space_str<<status.tas_test_ratio<<space_str<<status.hagl_test_ratio
                       <<space_str<<status.beta_test_ratio<<space_str<<status.time_slip<<space_str<<status.gps_check_fail_flags<<space_str<<status.filter_fault_flags
                      <<space_str<<status.innovation_check_flags<<space_str<<status.solution_status_flags<<space_str<<status.nan_flags<<space_str<<status.health_flags
                     <<space_str<<status.timeout_flags<<space_str<<status.pre_flt_fail<<std::endl;
    //         std::cout<<"timestamp"<<status.timestamp<<std::endl;
    //         std::cout<<status.timestamp<<space_str<<states_str<<space_str<<status.n_states<<status.vibe[0]
    //                 <<space_str<<status.vibe[1]<<space_str<<status.vibe[2]<<space_str<<cov_str<<space_str<<status.control_mode_flags<<space_str
    //                <<status.pos_horiz_accuracy<<space_str<<status.pos_vert_accuracy<<space_str<<status.mag_test_ratio<<space_str<<status.vel_test_ratio
    //               <<space_str<<status.pos_test_ratio<<space_str<<status.hgt_test_ratio<<space_str<<status.tas_test_ratio<<space_str<<status.hagl_test_ratio
    //              <<space_str<<status.beta_test_ratio<<space_str<<status.time_slip<<space_str<<status.gps_check_fail_flags<<space_str<<status.filter_fault_flags
    //             <<space_str<<status.innovation_check_flags<<space_str<<status.solution_status_flags<<space_str<<status.nan_flags<<space_str<<status.health_flags
    //            <<space_str<<status.timeout_flags<<space_str<<status.pre_flt_fail<<std::endl;
    //   std::cout<<"mag"<<status.mag_test_ratio<<std::endl;
    //   std::cout<<"state[24]:"<<status.states[23]<<"state_var[24]:"<<status.covariances[23]<<std::endl;
            //ECL_WARN("status.timestamp%d,mag_test_ratio%f，vel_test_ratio%f",status.timestamp,status.mag_test_ratio,status.vel_test_ratio);
            //ECL_WARN("control_mode_flags%d",status.control_mode_flags);
            //estimate_status_out<<ekfStatusStream.str()<<std::endl;



           {
                // publish estimator innovation data
                ekf2_innovations_s innovations;
                innovations.timestamp = now;
                _ekf.get_vel_pos_innov(&innovations.vel_pos_innov[0]);
                _ekf.get_aux_vel_innov(&innovations.aux_vel_innov[0]);
                _ekf.get_mag_innov(&innovations.mag_innov[0]);
                _ekf.get_heading_innov(&innovations.heading_innov);
                _ekf.get_airspeed_innov(&innovations.airspeed_innov);
                _ekf.get_beta_innov(&innovations.beta_innov);
                _ekf.get_flow_innov(&innovations.flow_innov[0]);
                _ekf.get_hagl_innov(&innovations.hagl_innov);
                _ekf.get_drag_innov(&innovations.drag_innov[0]);

                _ekf.get_vel_pos_innov_var(&innovations.vel_pos_innov_var[0]);
                _ekf.get_mag_innov_var(&innovations.mag_innov_var[0]);
                _ekf.get_heading_innov_var(&innovations.heading_innov_var);
                _ekf.get_airspeed_innov_var(&innovations.airspeed_innov_var);
                _ekf.get_beta_innov_var(&innovations.beta_innov_var);
                _ekf.get_flow_innov_var(&innovations.flow_innov_var[0]);
                _ekf.get_hagl_innov_var(&innovations.hagl_innov_var);
                _ekf.get_drag_innov_var(&innovations.drag_innov_var[0]);

                _ekf.get_output_tracking_error(&innovations.output_tracking_error[0]);

                //std::stringstream ekf_innovations_stream;
                std::string vel_pos_ino_str=mat2Str(innovations.vel_pos_innov,6);
                std::string mag_ino_str=mat2Str(innovations.mag_innov,3);
                std::string mag_ino_var_str=mat2Str(innovations.mag_innov_var,3);
                std::string vel_pos_var_ino_str=mat2Str(innovations.vel_pos_innov_var,6);
                std::string output_tra_ero_str=mat2Str(innovations.output_tracking_error,3);
                ekf_innovations_out<<innovations.timestamp<<space_str<<vel_pos_ino_str<<space_str<<mag_ino_str
                                     <<space_str<<innovations.heading_innov<<space_str<<innovations.airspeed_innov
                                    <<space_str<<innovations.beta_innov<<space_str<<innovations.flow_innov[0]<<space_str
                                   <<innovations.flow_innov[1]<<space_str<<innovations.hagl_innov<<space_str<<vel_pos_var_ino_str<<space_str
                                  <<mag_ino_var_str<<space_str<<innovations.heading_innov_var<<space_str<<innovations.airspeed_innov_var
                                 <<space_str<<innovations.beta_innov_var<<space_str<<innovations.flow_innov_var[0]<<space_str<<innovations.flow_innov_var[1]
                                <<space_str<<innovations.hagl_innov_var<<space_str<<output_tra_ero_str<<space_str
                               <<innovations.drag_innov[0]<<space_str<<innovations.drag_innov[1]<<space_str<<innovations.drag_innov_var[0]<<space_str<<innovations.drag_innov_var[1]
                              <<space_str<<innovations.aux_vel_innov[0]<<space_str<<innovations.aux_vel_innov[1]<<std::endl;
                // std::cout<<"track_erro"<<output_tra_ero_str<<std::endl;
                //ekf_innovations_out<<ekf_innovations_stream.str()<<std::endl;


                // calculate noise filtered velocity innovations which are used for pre-flight checking
                //判断的部分在组合导航分析中意义不大，故不移植，会影响position的有效标识






             //TODO: uORB definition does not define what these variables are. We have assumed them to be horizontal and vertical 1-std dev accuracy in metres
            matrix::Vector3f pos_var, vel_var;
            _ekf.get_pos_var(pos_var);
            _ekf.get_vel_var(vel_var);
            //ECL_INFO("pos_var: %lf,%lf,%lf\n", pos_var(0), pos_var(1), pos_var(2) );
            //ECL_INFO("vel_var: %lf,%lf,%lf\n", vel_var(0), vel_var(1), vel_var(2) );

            }

        }
    // ECL_INFO("end\n");


    }
}

const Vector3f Ekf2::get_vel_body_wind()
{
    // Used to correct baro data for positional errors

    matrix::Quatf q;
    _ekf.copy_quaternion(q.data());
    matrix::Dcmf R_to_body(q.inversed());

    // Calculate wind-compensated velocity in body frame
    // Velocity of body origin in local NED frame (m/s)
    float velocity[3];
    _ekf.get_velocity(velocity);

    float velNE_wind[2];
    _ekf.get_wind_velocity(velNE_wind);

    Vector3f v_wind_comp = {velocity[0] - velNE_wind[0], velocity[1] - velNE_wind[1], velocity[2]};

    return R_to_body * v_wind_comp;
}



int main(int argc, char *argv[])
{
    ECL_INFO("begin\n");
	bReadGPS = true;
	Ekf2* _ekf2 = new Ekf2();
    _ekf2->print_status();
    _ekf2->task_main();

    return 0;
}
