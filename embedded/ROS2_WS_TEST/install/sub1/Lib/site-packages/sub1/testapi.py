import shared_data

def check_vehicle_number():
    print(shared_data.vehicle_number)

    stored_vn =  shared_data.vehicle_number
    carnum = "11가1234"

    if(carnum == stored_vn):
        print("ok")
    else:
        print("no")

    # return app.state.vehicle_number

