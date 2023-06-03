    if ( * caracter == 'p') {
      sendString("Product mode selected.\r\n");
  	  int count = 0;
  	  int countLoud = 2;
  	  quietMode(); 
  	  mode = 1; 

      while (1) {

    	int rv = readVoltage();

        if ( * caracter == 'd') {
          break;
        } else if (rv >= 3000) {
        	if (mode == 3) { 
        		countLoud = countLoud - 1; 
        	} else if (mode == 2) { 
        		count = count - 1;
        		if (count == 0) {
        			quietMode();
        			mode = 1;
        		}
        	}
        	if (countLoud == 0) { 
        		quietMode(); 
        		mode = 1; 
        		countLoud = 2; 
        		count = 0; 
        	}

        } else {
        	count = count + 1; 
        	if (count == 5) { 
        		loudMode(); 
        		mode = 3; 
        	} else if (count == 2) {
        		warningMode(); 
            	mode = 2; 

        	}
        }

        int mcurv = rv / 16;

        if (mcurv < 8) {
        	mcurv = 9;
        }