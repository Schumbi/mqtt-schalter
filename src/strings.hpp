#ifndef MAKELIGHT_STRINGS_H
#define MAKELIGHT_STRINGS_H

#include <Arduino.h>

// ab in PROGMEM damit Todo

String Response = "";

String html_anfang = "<!DOCTYPE html>\r\n<html>\r\n\
<head>\r\n<meta content=\"text/html; charset=ISO-8859-1\" http-equiv=\"content-type\">\r\n\
<title>WebSchalter</title>\r\n<body><p>";
String html_ende = "</p></body>\r\n</html>";

String redirect = "<!DOCTYPE html>\r\n<html>\r\n\
<head>\r\n\
<meta content=\"text/html; charset=ISO-8859-1\" \
<meta http-equiv=\"refresh\" content=\"0; URL='./'\" /> \
</head><body/></html>";

String form = "<form action='led'><input type='radio' name='state' value='1' checked>On<input type='radio' name='state' value='0'>Off<input type='submit' value='Submit'></form>";

#endif // MAKELIGHT_STRINGS_H
