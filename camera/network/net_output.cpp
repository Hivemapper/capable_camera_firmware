/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * net_output.cpp - send output over network.
 */

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "net_output.hpp"

NetOutput::NetOutput(VideoOptions const *options) : Output(options)
{
	char protocol[4];
	char sock_path[20];
	std::string address;

	int start, end, a, b, c, d, port;
	unix_socket_ = false;

	sscanf(options->output.c_str(), "%3s://", protocol);
	
	if (strcmp(protocol, "udp") == 0 || strcmp(protocol, "tcp") == 0) {
		if (sscanf(options->output.c_str(), "%3s://%n%d.%d.%d.%d%n:%d", protocol, &start, &a, &b, &c, &d, &end, &port) != 6)
			throw std::runtime_error("bad network address " + options->output);
		address = options->output.substr(start, end - start);
	} else {
		sscanf(options->output.c_str(), "%3s://%n%s%n", protocol, &start, sock_path, &end);
	}

	if (strcmp(protocol, "udp") == 0)
	{
		saddr_ = {};
		saddr_.sin_family = AF_INET;
		saddr_.sin_port = htons(port);
		if (inet_aton(address.c_str(), &saddr_.sin_addr) == 0)
			throw std::runtime_error("inet_aton failed for " + address);

		fd_ = socket(AF_INET, SOCK_DGRAM, 0);
		if (fd_ < 0)
			throw std::runtime_error("unable to open udp socket");

		saddr_ptr_ = (const sockaddr *)&saddr_; // sendto needs these for udp
		sockaddr_in_size_ = sizeof(struct sockaddr_in);
	}
	else if (strcmp(protocol, "tcp") == 0)
	{
		// WARNING: I've not actually tried this yet...
		if (options->listen)
		{
			// We are the server.
			int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
			if (listen_fd < 0)
				throw std::runtime_error("unable to open listen socket");

			sockaddr_in server_saddr = {};
			server_saddr.sin_family = AF_INET;
			server_saddr.sin_addr.s_addr = INADDR_ANY;
			server_saddr.sin_port = htons(port);

			if (bind(listen_fd, (struct sockaddr *)&server_saddr, sizeof(server_saddr)) < 0)
				throw std::runtime_error("failed to bind listen socket");
			listen(listen_fd, 1);

			if (options->verbose)
				std::cerr << "Waiting for client to connect..." << std::endl;
			fd_ = accept(listen_fd, (struct sockaddr *)&saddr_, &sockaddr_in_size_);
			if (fd_ < 0)
				throw std::runtime_error("accept socket failed");
			if (options->verbose)
				std::cerr << "Client connection accepted" << std::endl;

			close(listen_fd);
		}
		else
		{
			// We are a client.
			saddr_ = {};
			saddr_.sin_family = AF_INET;
			saddr_.sin_port = htons(port);
			if (inet_aton(address.c_str(), &saddr_.sin_addr) == 0)
				throw std::runtime_error("inet_aton failed for " + address);

			fd_ = socket(AF_INET, SOCK_STREAM, 0);
			if (fd_ < 0)
				throw std::runtime_error("unable to open client socket");

			if (options->verbose)
				std::cerr << "Connecting to server..." << std::endl;
			if (connect(fd_, (struct sockaddr *)&saddr_, sizeof(sockaddr_in)) < 0)
				throw std::runtime_error("connect to server failed");
			if (options->verbose)
				std::cerr << "Connected" << std::endl;
		}

		saddr_ptr_ = NULL; // sendto doesn't want these for tcp
		sockaddr_in_size_ = 0;
	}
	else if (strcmp(protocol, "sck") == 0)
	{
		unix_socket_ = true;
		sock_ = {};
    	sock_.sun_family = AF_UNIX;
    	strncpy(sock_.sun_path, sock_path, end-start);

    	fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
		if (fd_ < 0) {
			throw std::runtime_error("unable to open unix socket");
		}

		if (connect(fd_, (struct sockaddr *) &sock_, sizeof(struct sockaddr_un)) == -1) {
      		throw std::runtime_error("unable to connect to unix socket");
    	}
	}
	else
		throw std::runtime_error("unrecognised network protocol " + options->output);
}

NetOutput::~NetOutput()
{
	close(fd_);
}

// Maximum size that sendto will accept.
constexpr size_t MAX_UDP_SIZE = 65507;

char EOL[] = {'\r', '\n'};

int count_digits(int x)  
{  
    x = abs(x);  
    return (x < 10 ? 1 :   
        (x < 100 ? 2 :   
        (x < 1000 ? 3 :   
        (x < 10000 ? 4 :   
        (x < 100000 ? 5 :   
        (x < 1000000 ? 6 :   
        (x < 10000000 ? 7 :  
        (x < 100000000 ? 8 :  
        (x < 1000000000 ? 9 :  
        10)))))))));  
}  

void NetOutput::outputUnixSocket(void *mem, size_t size, int64_t timestamp_us, uint32_t /*flags*/)
{
	int ret = 0;

	// TODO : don't hard code the static length of the header here
	size_t header_length = 17+count_digits(size);

	// Prepare the header string with topic and number of bytes that follow the line break
	char header[100] = "";
	sprintf(header, "PUB frame.jpeg %lu\r\n", static_cast<unsigned long>(size));

	// if (options_->verbose)
	// 	std::cerr << "NetOutput: output buffer " << mem << " size " << size << "\n";

	if (write(fd_, header, header_length) < 0) {
		throw std::runtime_error("failed to send data on unix socket");
	}

	if ((ret = write(fd_, mem, size)) < 0) {
		throw std::runtime_error("failed to send data on unix socket");
	}

	if (write(fd_, &EOL, 2) < 0) {
		throw std::runtime_error("failed to send data on unix socket");
	}

	// if (options_->verbose)
	// 	std::cerr << "  wrote " << ret << "\n";
}

void NetOutput::outputBuffer(void *mem,
                             size_t size,
                             void *prevMem,
                             size_t prevSize,
                             int64_t timestamp_us,
                             uint32_t /*flags*/)
{
  (void)prevMem;
  (void)prevSize;

	if (unix_socket_) {
		outputUnixSocket(mem, size, timestamp_us, 0);
		return;
	}

	struct msghdr msg = {};
	struct iovec iov[3] = {{}, {}, {}};
	int ret = 0;

	size_t max_size = saddr_ptr_ ? MAX_UDP_SIZE : size;

	if (options_->verbose)
		std::cerr << "NetOutput: output buffer " << mem << " size " << size << "\n";
	
	// TODO : don't hard code the static length of the header here
	size_t header_length = 17+count_digits(size);

	// Prepare the header string with topic and number of bytes that follow the line break
	char header[100] = "";
	sprintf(header, "PUB frame.jpeg %lu\r\n", static_cast<unsigned long>(size));

	size_t bytes_to_send = std::min(size, max_size - header_length);
	uint8_t *ptr = (uint8_t *)mem;

	// First, we create a composite message which combined the header with the start of the image data
	// This is a bit more complicated than two calls to `sendto`, but reduced the number of packets
	iov[0].iov_base = &header;
	iov[0].iov_len = header_length;

	iov[1].iov_base = ptr;
	iov[1].iov_len = bytes_to_send;

	msg.msg_iovlen = 2;

	// If the payload, header, and EOL butes can fit in a single message, do so (this is very unlikely)
	if (max_size - header_length - bytes_to_send >= 2) {
		iov[2].iov_base = &EOL;
		iov[2].iov_len = 2;

		msg.msg_iovlen = 3;
	} 
	
	msg.msg_iov = &iov[0];
	msg.msg_control = NULL;
	msg.msg_controllen = 0;
	msg.msg_flags = 0;

	// Sed the destination address and port for the packet
	msg.msg_name = &saddr_;
	msg.msg_namelen = sockaddr_in_size_;
	
	// Send the composite packet containing header and start of the image data
	if ((ret = sendmsg(fd_, &msg, 0)) < 0) {
		std::cerr << "sendmsg err " << ret << "\n";
		throw std::runtime_error("failed to send data on socket");
	}

	// Advance data trackign what part of the image we've sent already
	ptr += bytes_to_send;
	size -= bytes_to_send;

	// Until the last packet, bytes_to_send will be constant 
	bytes_to_send = std::min(size, max_size);

	// Send image data until we have less than `max_size` left to send
	while (size >= max_size) {
		if (sendto(fd_, ptr, bytes_to_send, 0, saddr_ptr_, sockaddr_in_size_) < 0) {
			throw std::runtime_error("failed to send data on socket");
		}

		ptr += bytes_to_send;
		size -= bytes_to_send;
	}

	// Create the final packet, which will be a composite of 
	// - the remainder of the image data
	// - the EOL bytes
	iov[0].iov_base = ptr;
	iov[0].iov_len = size;

	iov[1].iov_base = &EOL;

	// Check to make sure that we have two bytes available to put the EOL in
	if (max_size - size < 2) {
		iov[1].iov_len = 1;
	} else {
		iov[1].iov_len = 2;
	}

	msg.msg_iovlen = 2;

	// Send the composite packet
	if ((ret = sendmsg(fd_, &msg, 0)) < 0) {
		std::cerr << "sendmsg err " << ret << "\n";
		throw std::runtime_error("failed to send data on socket");
	}

	// If size + 1 happens to match max_length, then the EOL bytes will split accross two packets
	// Here, we send the last EOL byte if that occurs
	if (max_size - size < 2) {
		sendto(fd_, &EOL[1], 1, 0, saddr_ptr_, sockaddr_in_size_);
	}
	
}
