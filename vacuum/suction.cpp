#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <bitset>

#define debug if (1) std::cout 

class AirSensing
{
  public:
    AirSensing();
    ~AirSensing() { };
    bool readPressureOnce(int& value);
    
  private:
    std::string port_name_;
    int baud_rate_;
    std::string cmd_read_;
    double threshold_;

    boost::asio::io_service iosev_;
    std::shared_ptr<boost::asio::serial_port> sp_;

    int hex2decimal(char* hexstr);
    static void received_cb(bool& data_available, 
                std::shared_ptr<boost::asio::deadline_timer> timeout_timer, 
                const boost::system::error_code& error, 
                std::size_t bytes_transferred);
    static void timeout_cb(boost::asio::serial_port& serial_port, const boost::system::error_code& error);
    bool loadConfig(std::string config_file_path);

};

AirSensing::AirSensing()
{
    // loadConfig(config_file_path);
    baud_rate_ = 9600;
    port_name_ = "/dev/ttyUSB0";
    cmd_read_ = "";
    threshold_ = 10;

    sp_ = std::make_shared<boost::asio::serial_port>(iosev_, port_name_);
    sp_->set_option(boost::asio::serial_port::baud_rate(baud_rate_));
    sp_->set_option(
        boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    sp_->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    sp_->set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::two));
    sp_->set_option(boost::asio::serial_port::character_size(8));
}

bool AirSensing::readPressureOnce(int& value)
{
    unsigned char  sp_buffer_[7];
    bool data_available = false;

    std::fill_n(sp_buffer_, 7, 0x00);

    std::vector<char> bytes;
    // hex to bytes
    for (unsigned int i = 0; i < cmd_read_.length(); i += 2)
    {
        std::string byteString = cmd_read_.substr(i, 2);
        char byte = (char)strtol(byteString.c_str(), NULL, 16);
        bytes.push_back(byte);
    }
    boost::asio::write(*sp_, boost::asio::buffer(bytes,8));

    //100ms waiting time
    usleep(100000);
    // printf("receiving cmd \n");

    // start read and count time
    std::shared_ptr<boost::asio::deadline_timer> timeout_timer_ = std::make_shared<boost::asio::deadline_timer>(iosev_);

    sp_->async_read_some(boost::asio::buffer(sp_buffer_),
    boost::bind(&received_cb, boost::ref(data_available), timeout_timer_,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
    timeout_timer_->expires_from_now(boost::posix_time::milliseconds(100));
    timeout_timer_->async_wait(boost::bind(&timeout_cb, boost::ref(*sp_),
                    boost::asio::placeholders::error));

    iosev_.run();  // will block until async callbacks are finished
    iosev_.reset();

    if(data_available)
    {
        // bytes to hex
        std::stringstream stream;
        for(unsigned int i=0;i<sizeof(sp_buffer_);i++)
            {
                stream<<std::hex<<static_cast<int>(sp_buffer_[i])<<",";  //output HEX
            }
        std::string recv_str(stream.str());    

        // get the hex of air pressure value
        int iCount = 0;
        std::vector<unsigned int> comma_index;
        for(unsigned int i = 0; i < recv_str.size(); i++)
        {
                if(',' == recv_str[i])
                {
                    iCount++;
                    if(iCount ==3 ||iCount ==4 ||iCount ==5 )
                    {
                        comma_index.push_back(i);
                    }
                }
        }
        std::string hex_air_value = recv_str.substr((comma_index[0]+1),(comma_index[1]-comma_index[0]-1))+recv_str.substr((comma_index[1]+1),(comma_index[2]-comma_index[1]-1));
        // debug<<hex_air_value<<std::endl;
        
        // convert hex to Decimal from signed 2's complement
        char * strhex = const_cast<char*>(hex_air_value.c_str());
        int result = hex2decimal(strhex);
        // debug<< "air pressure in 0.1Kpa: " <<  result<<std::endl;
        value = result;
        
        return true;
    }
    value = 0;
    return false;   
}

// hex to decimal from signed 2's complement
int AirSensing::hex2decimal(char* hexstr)
{
    if(hexstr ==NULL)
        return 0;
    char binary[16]={0};
    //Convert strings to a long-integer value.
    long i32 = strtol(hexstr,NULL,16);
    std::string binary_str = std::bitset<16>(i32).to_string(); //to binary
    std::copy(binary_str.begin(), binary_str.end(), binary);

    int toint = 0;
    int ratio = 1;
    if(hexstr[0]>'8')
    {
        for(int i=1;i<16;i++) // dosomething
        {
            binary[i] ='0'+!(binary[i]-'0');
        }

        int nTakeover = 0;
        bool isoverflow = false;
        for(int i=15;i>=0;i--)
        {
            int nsum = binary[i]-'0'+nTakeover; 
            if(i==15)
                nsum++;
            if(nsum==2)
            {
                if(i == 0)
                    isoverflow = true;
                else
                {
                    nsum-=2;
                    nTakeover = 1;
                    binary[i] = '0'+nsum;
                }
            }
            else
            {
                binary[i] = '0'+nsum;
                break;
            }
        }
        for(int j=15;j>0;--j)
        {
            toint = toint+ (binary[j]-'0')*ratio;
            ratio=ratio*2;
        }
        toint = toint*(-1);
        return toint;
    }
    else
    {
        toint= i32;
        return toint;
    }
}

// receive call back
void AirSensing::received_cb(bool& data_available, 
                std::shared_ptr<boost::asio::deadline_timer> timeout_timer, 
                const boost::system::error_code& error, 
                std::size_t bytes_transferred)
{
    // debug << "im in receive cb." << std::endl;
  if (error || !bytes_transferred)
  {
    // No data was read!
    data_available = false;
    debug << "no data received." << std::endl;

    return;
  }

  timeout_timer->cancel();
  data_available = true;
}

// timer call back
void AirSensing::timeout_cb(boost::asio::serial_port& serial_port, const boost::system::error_code& error)
{
  if (error)
  {
    //   debug << "[timer] data received, cancel current timer." << std::endl;
    // Data was read and this timeout was canceled
    return;
  }
    debug << "[timer] timeout ,cancel current reading task." << std::endl;

    serial_port.cancel();  // will cause read_callback to fire with an error
}




//**********************************************
//*  TEST Airsensing with timeout
//*  To build: 
//*  g++ -o test AirSensing.cpp -lpthread -I /usr/include/boost/asio
int main(int argc, char* argv[])
{
    AirSensing sensor;
    int curr_pressure = 0;
    for (int i = 0; i < 1000; ++i)
    {
        if ( !sensor.readPressureOnce(curr_pressure) )
        {
            debug << "failed to read pressure." << std::endl;
            continue;
        }
        debug << "idx " << i << ", current pressure " << curr_pressure << std::endl;    
    }

    return 0;
}