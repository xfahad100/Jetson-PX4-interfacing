#include "mavlink_interface.h"
MavlinkInterface::MavlinkInterface()
    :
      serial_enabled_(true),
      enable_lockstep_(true),
      m_status{},
      m_buffer{},
      io_service(),
      serial_dev(io_service),
      device_(kDefaultDevice),
      baudrate_(kDefaultBaudRate),
      rx_buf{} {
    Load();
}
MavlinkInterface::~MavlinkInterface() { close(); }

void MavlinkInterface::Load() {
    serial_enabled_ =true;
    std::cout<<"in load"<<std::endl;
  if (serial_enabled_) {
    // Set up serial interface
    io_service.post(std::bind(&MavlinkInterface::do_read, this));
    // run io_service for async io
    io_thread = std::thread([this]() { io_service.run(); });
    open();

  }
}


void MavlinkInterface::open() {
  try {
    serial_dev.open(device_);
    serial_dev.set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
    serial_dev.set_option(boost::asio::serial_port_base::character_size(8));
    serial_dev.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_dev.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_dev.set_option(
        boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    std::cout << "Opened serial device " << device_ << "\n";
  } catch (boost::system::system_error &err) {
    std::cerr << "Error opening serial device: " << err.what() << "\n";
  }
}

void MavlinkInterface::do_read(void) {
  serial_dev.async_read_some(boost::asio::buffer(rx_buf),
                             boost::bind(&MavlinkInterface::parse_buffer, this, boost::asio::placeholders::error,
                                         boost::asio::placeholders::bytes_transferred));
 // std::cout<<boost::asio::placeholders::bytes_transferred<<std::endl;
}

// Based on MAVConnInterface::parse_buffer in MAVROS
void MavlinkInterface::parse_buffer(const boost::system::error_code &err, std::size_t bytes_t) {

    mavlink_message_t massg;
    mavlink_status_t _status;

    //uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    // check message is write length
   // unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &massg);
   // std::cout << bytes_t <<std::endl;


    for (auto i = 0; i < bytes_t ; i++) {
       //  std::cout << "buf::  " << unsigned( _status.packet_rx_drop_count) <<std::endl;
        uint8_t c = rx_buf[i];
       //std::cout << unsigned(c) <<std::endl;
                    if (mavlink_parse_char(MAVLINK_COMM_0, c, &massg, &_status)) {

                      //  mavlink_scaled_imu_t imu1;
                        //mavlink_message_t imurev;
     // std::lock_guard<std::recursive_mutex> lock(mutex);
    //                    mavlink_message_t msgpx;
                        //mavlink_heartbeat_t packet1;
                       // mavlink_msg_heartbeat_decode(&massg, &packet1);
                        std::cout << "msgid::  " << unsigned(massg.msgid) <<std::endl;  // Pixhawk sysid
                        //std::cout << "comp::  " <<std::endl;



    //usleep(20000);
    switch(massg.msgid){
      case MAVLINK_MSG_ID_HIGHRES_IMU:
    {
    mavlink_highres_imu_t imuhighres;
    mavlink_msg_highres_imu_decode(&massg,&imuhighres);
    armack = imuhighres.xacc;
    std::cout << "imu::  " << imuhighres.xacc <<std::endl;
    break;
    }
      case MAVLINK_MSG_ID_ATTITUDE:
      {
        mavlink_attitude_t at1;
    mavlink_msg_attitude_decode(&massg,&at1);
      //current = at1.time_boot_ms;
   //   gps_input[0] = (1/float((current - last)))*1000;
      //std::cout << "hertz::  " << (1/float((current - last)))*1000 <<endl;
  // std::cout << "pitchmav::  " << float(input_reference_2[0]) <<endl; //radians
//      std::cout << "roll::  " << float(input_reference_2[1]) <<endl;
//      std::cout << "yaw::  " << float(input_reference_2[2]) <<endl;
        break;
      }
      case MAVLINK_MSG_ID_HEARTBEAT:
    {
        mavlink_heartbeat_t msghb;
        mavlink_msg_heartbeat_decode(&massg,&msghb);
      //  armed_ = (msghb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
    //std::cout << "armstate::  " << msghb.base_mode <<endl;
        break;
    }
      case MAVLINK_MSG_ID_BATTERY_STATUS:
    {
        mavlink_battery_status_t battery;
        mavlink_msg_battery_status_decode(&massg,&battery);
        //gps_input[0]= double(battery.battery_remaining);
       // cout << "batteryvolt0::  " << double((unsigned(battery.voltages[0]))/1000) <<endl;
       // cout << "batteryvolt1::  " << double((unsigned(battery.voltages[1]))/1000) <<endl;
        break;
    }
      case MAVLINK_MSG_ID_GPS_RAW_INT:
    {
    mavlink_gps_raw_int_t gpsraw;
    mavlink_msg_gps_raw_int_decode(&massg,&gpsraw);
  //  cout << "gpsvisible::  " << unsigned(gpsraw.satellites_visible) <<endl;
    //gps_input[0]=gpsraw.lat / (double)1E7;
    //gps_input[1]=gpsraw.lon /(double)1E7;
    //gps_input[2]=gpsraw.alt /(double)1E7;
    break;
    }
      case MAVLINK_MSG_ID_VFR_HUD:
    {
     mavlink_vfr_hud_t Airpressure;
     mavlink_msg_vfr_hud_decode(&massg,&Airpressure);

    // airspeed[2] = servo.controls[2];
//     std::cout << " servo 3::  " << airspeed[2] <<endl;
//     std::cout << " servo 4::  " << servo.controls[3] <<endl;
//    std::cout << " servo 5::  " << servo.controls[4] <<endl;
     break;
    }
      case MAVLINK_MSG_ID_COMMAND_ACK:
    {
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(&massg, &ack);

       // std::cout << " armstate::  " << armack <<endl;
        break;
    }
      case MAVLINK_MSG_ID_ODOMETRY:
    {
        mavlink_odometry_t odometry;
        mavlink_msg_odometry_decode(&massg,&odometry);

        break;
    }

    }                    }
    }

  do_read();
}

void MavlinkInterface::close() {
  if (serial_enabled_) {

    io_service.stop();
    serial_dev.close();

    if (io_thread.joinable()) io_thread.join();

  }
}

float MavlinkInterface::getIMU(){
    return armack;
}
