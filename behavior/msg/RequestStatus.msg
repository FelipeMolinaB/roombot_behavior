uint16  request_id
uint8   status
uint8   PENDING   = 0   # The request was received and is waiting to be processed
uint8   ACTIVE    = 1   # if
uint8   SUCCEED   = 2   # if type == delivery the request will completed when the
                        # guest confirm the delivery, if type == pick_up the request
                        # will completed when the roombot deliver the guest's object
                        # in the delivery room
uint8   CANCELED  = 3   # the request was canceled by the guests
