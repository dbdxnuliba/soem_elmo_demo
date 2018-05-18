#ifndef EXCEPTIONS_H
#define EXCEPTIONS_H

#include <exception>
#include <boost/exception/all.hpp>
#include <boost/format.hpp>


class Exception : public std::runtime_error{
public:
  Exception(const std::string &w) : std::runtime_error(w){}
};

class PointerInvalid : public Exception{
public:
  PointerInvalid(const std::string &w) : Exception("Pointer Invalid"){}
};

class ParseException : public Exception{
public:
  ParseException(const std::string &w) : Exception(w){}
};

class TimeoutException : public Exception{
public:
  TimeoutException(const std::string &w) : Exception(w){}
};

#endif // EXCEPTIONS_H
