// Copyright 2011 Juri Glass, Mathias Runge, Nadim El Sayed
// DAI-Labor, TU-Berlin
//
// This file is part of libSML.
//
// libSML is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// libSML is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with libSML.  If not, see <http://www.gnu.org/licenses/>.

#ifndef SML_GET_LIST_REQUEST_H_
#define SML_GET_LIST_REQUEST_H_

#include "sml_list.h"
#include "sml_octet_string.h"
#include "sml_shared.h"
#include "sml_time.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	octet_string *client_id;
	octet_string *server_id; // optional
	octet_string *username;  // optional
	octet_string *password;  // optional
	octet_string *list_name; // optional
} sml_get_list_request;

sml_get_list_request *sml_get_list_request_init();
sml_get_list_request *sml_get_list_request_parse(sml_buffer *buf);
void sml_get_list_request_write(sml_get_list_request *msg, sml_buffer *buf);
void sml_get_list_request_free(sml_get_list_request *msg);

#ifdef __cplusplus
}
#endif

#endif /* SML_GET_LIST_REQUEST_H_ */
