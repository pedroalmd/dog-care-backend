	   if ( * caracter == 'p') {
	      sendString("Product mode selected.\r\n");

	      while (1) {
	    	int rv = readVoltage();

	        if ( * caracter == 'd') {
	          break;
	        } else if (rv >= 3000) {
	          loudMode();
	          mode = 3;

	        } else if (rv < 1500) {
            quietMode();
	          mode = 1;

	        } else {
	          warningMode();
	          mode = 2;

	        }