/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * net_output.hpp - send output over network.
 */

#pragma once

#include <netinet/in.h>
#include <sys/un.h>

#include "output.hpp"

class NetOutput : public Output
{
public:
	NetOutput(VideoOptions const *options);
	~NetOutput();

protected:
	void outputUnixSocket(void *mem, size_t size, int64_t timestamp_us, uint32_t flags);
	void outputBuffer(void *mem, size_t size, int64_t timestamp_us, uint32_t flags) override;

private:
	int fd_;
	bool unix_socket_;
	sockaddr_in saddr_;
	sockaddr_un sock_;
	const sockaddr *saddr_ptr_;
	socklen_t sockaddr_in_size_;
};
