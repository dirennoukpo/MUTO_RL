#pragma once

#include <stdexcept>
#include <string>

#include "muto_link/export.hpp"

namespace muto_link {

class MUTO_LINK_API Error : public std::runtime_error {
public:
    explicit Error(const std::string& message) : std::runtime_error(message) {}
};

class MUTO_LINK_API TransportError : public Error {
public:
    explicit TransportError(const std::string& message) : Error(message) {}
};

class MUTO_LINK_API ProtocolError : public Error {
public:
    explicit ProtocolError(const std::string& message) : Error(message) {}
};

class MUTO_LINK_API TimeoutError : public TransportError {
public:
    explicit TimeoutError(const std::string& message) : TransportError(message) {}
};

class MUTO_LINK_API ValidationError : public ProtocolError {
public:
    explicit ValidationError(const std::string& message) : ProtocolError(message) {}
};

} // namespace muto_link
