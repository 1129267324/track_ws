#ifndef YLOG_YLOG_H_
#define YLOG_YLOG_H_

#include <string>
#include <fstream>
#include <cassert>
#include <ctime>

class YLog{
 private:
  std::ofstream of_;
  int minlevel_;
 public:
  enum Type {
    ADD = 0,
    OVER
  };
  enum Level {
    INFO = 0,
    ERR
  };
  YLog(const int level, const std::string &logfile, const int type = YLog::OVER) : minlevel_(level) {
    assert((this->ERR == level || this->INFO == level) && "Logfile create failed, please check the level(YLog::ERR or YLog::INFO.");
    if (type == this->ADD) {
      this->of_.open(logfile.c_str(),std::ios_base::out|std::ios_base::app);
    } else if (type == this->OVER) {
      this->of_.open(logfile.c_str(),std::ios_base::out|std::ios_base::trunc);
    } else {
      assert(0 && "Logfile create failed, please check the type(YLog::OVER or YLog::ADD).");
    }
    assert(this->of_.is_open() && "Logfile create failed, please check the logfile's name and path.");
    return;
  }
  ~YLog(){
    if (this->of_.is_open()) {
      this->of_.close();
    }
    return;
  }
  template<typename T> void W(const std::string &codefile, const int codeline, const int level, const std::string &info, const T &value) {
    assert(this->of_.is_open() && "Logfile write failed.");
    if (this->ERR == level) {
      this->of_ << "[ERROR] ";
    } else if (this->INFO == level) {
      if (this->INFO == this->minlevel_) {
        this->of_ << "[INFO] ";
      } else {
        return;
      }
    } else {
      assert(0 && "Log write failed, please check the level(YLog::ERR or YLog::INFO.");
    }
    time_t sectime = time(NULL);
    tm tmtime;
#ifdef _WIN32
#if _MSC_VER<1600
    tmtime = *localtime(&sectime);
#else
    localtime_s(&tmtime, &sectime);
#endif
#else
    localtime_r(&sectime, &tmtime);
#endif
    this->of_ << tmtime.tm_year+1900 << '-' << tmtime.tm_mon+1 << '-' << tmtime.tm_mday <<
      ' ' << tmtime.tm_hour << ':' << tmtime.tm_min << ':' << tmtime.tm_sec <<
      ' '<< codefile << '(' << codeline << ") " << info << " : " << value << std::endl;
    return;
  }
  // hex
  void W(const std::string &codefile, const int codeline, const int level, const std::string &info, const unsigned char *pBuf, int len) 
  {
    std::string strValue;
    char ch[10] = {0};
    for(int i = 0; i < len; i++)
    {
      sprintf(ch, "%02x ", pBuf[i]);
      strValue.append(ch);
    }
    W(codefile, codeline, level, info, strValue);
  }

  //  double
  void w_double_arr(const std::string &codefile, const int codeline, const int level, const std::string &info, const double *pBuf, int len, bool bwriteinfo = true)
  {
     std::string strValue;
    char ch[50] = {0};
    for(int i = 0; i < len; i++)
    {
      sprintf(ch, "%.6lf ", pBuf[i]);
      strValue.append(ch);
    }
    if(bwriteinfo){
      W(codefile, codeline, level, info, strValue); 
    }
    else
    {
      this->of_ << strValue << std::endl;      
    }

  }
  // int-double
  void w_int_double(int nVal, double fVal)
  {
    std::string strValue;
    strValue.append(std::to_string(nVal));
    strValue.append("    ");
    strValue.append(std::to_string(fVal));
    this->of_<<strValue<<std::endl;
    //this->of_.write(strValue.c_str(), strValue.length());
  }

};
#endif // YLOG_YLOG_H_ 
