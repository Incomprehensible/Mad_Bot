#include <esp_err.h>
#include <esp_log.h>
#include <esp_http_server.h>

#include "demo.h"

static char TAG[] = "HTTP SERVER";

// Build http body
const static char http_index_hml[] =
    "<html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\
    <title>Robot Control</title>\
    <style>\
        body {background-color: #e6f7ff; font-family: Arial, sans-serif; text-align: center;}\
        h1 {color: #007acc;}\
        .button {\
            display: inline-block;\
            padding: 20px 40px;\
            margin: 20px;\
            font-size: 24px;\
            color: white;\
            background-color: #007acc;\
            border: none;\
            border-radius: 10px;\
            cursor: pointer;\
            text-decoration: none;\
        }\
        .button:hover {background-color: #005f99;}\
    </style></head>\
    <body>\
    <h1>Control Panel</h1>\
    <a class=\"button\" href=\"/forward\">FORWARD</a><br>\
    <a class=\"button\" href=\"/backward\">BACKWARD</a><br>\
    <a class=\"button\" href=\"/rotate\">ROTATE</a><br>\
    <a class=\"button\" href=\"/break\">BREAK</a>\
    </body></html>";

/* Handler to send FORWARD command */
static esp_err_t forward_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "Received FORWARD request...");

	task_Demo(FORWARD);

    // redirect to index.html
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* Handler to send BACKWARD command */
static esp_err_t backward_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Received BACKWARD command...");

	task_Demo(BACKWARD);

    // redirect to index.html
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* Handler to send ROTATE command */
static esp_err_t rotate_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "Received ROTATE command...");

	task_Demo(ROTATE);

    // redirect to index.html
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* Handler to send BREAK command */
static esp_err_t break_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "Received BREAK command...");

	task_Demo(BREAK);

    // redirect to index.html
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* Handler to send index.html file */
static esp_err_t index_html_get_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "Received index.html request...");

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, http_index_hml, strlen(http_index_hml));
    return ESP_OK;
}

/* Function to start the HTTP command demo server */
esp_err_t start_http_demo_server()
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    /* Use the URI wildcard matching function in order to
     * allow the same handler to respond to multiple different
     * target URIs which match the wildcard scheme */
    config.uri_match_fn = httpd_uri_match_wildcard;

    ESP_LOGI(TAG, "Starting HTTP Server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start file server!");
        return ESP_FAIL;
    }

    /* URI handler for the FORWARD command */
    httpd_uri_t forward_cmd = {
        .uri       = "/forward",  // Match all URIs of type /path/to/file
        .method    = HTTP_GET,
        .handler   = forward_handler,
		.user_ctx  = NULL
        // .user_ctx  = server_data    // Pass server data as context
    };
    httpd_register_uri_handler(server, &forward_cmd);

    /* URI handler for the BACKWARD command */
    httpd_uri_t backward_cmd = {
        .uri       = "/backward",   // Match all URIs of type /upload/path/to/file
        .method    = HTTP_GET,
        .handler   = backward_handler,
		.user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &backward_cmd);

    /* URI handler for deleting files from server */
    httpd_uri_t rotate_cmd = {
        .uri       = "/rotate",   // Match all URIs of type /delete/path/to/file
        .method    = HTTP_GET,
        .handler   = rotate_handler,
        .user_ctx  = NULL    // Pass server data as context
    };
    httpd_register_uri_handler(server, &rotate_cmd);

	/* URI handler for the BREAK command */
	httpd_uri_t break_cmd = {
		.uri       = "/break",   // Match all URIs of type /delete/path/to/file
		.method    = HTTP_GET,
		.handler   = break_handler,
		.user_ctx  = NULL
	};
	httpd_register_uri_handler(server, &break_cmd);

	/* URI handler for the index.html file */
	httpd_uri_t index_html_cmd = {
		.uri       = "/",   // Match all URIs of type /delete/path/to/file
		.method    = HTTP_GET,
		.handler   = index_html_get_handler,
		.user_ctx  = NULL    // Pass server data as context
	};
	httpd_register_uri_handler(server, &index_html_cmd);

    return ESP_OK;
}