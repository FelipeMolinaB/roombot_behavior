uint16 request_id
uint8 floor
string room				      #Guest's Room
uint8 type				      #Request's type
uint8 DELIVERY  =1              #Guest requires a delivery
uint8 PICK_UP	  =2              #Guest requires a pick up
string[] request			          #Description of the guest's request ["item1;Quantity1",...,"itemN;QuantityN"]
