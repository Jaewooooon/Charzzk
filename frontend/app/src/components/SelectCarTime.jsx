import React, { useState, useEffect } from 'react';
import axios from 'axios';
import { useRecoilValue, useSetRecoilState, useRecoilState } from 'recoil'; 
import { accessTokenState } from '../recoil/LoginAtom'; 
import { parkingState } from '../recoil/parkingState.jsx';
import { currentIndexState } from '../recoil/CurrentIndex.jsx'; // currentIndexState 임포트
import '../components/styles/SelectCarTime.css'; 
import { useSwipeable } from 'react-swipeable';
import { batteryState } from '../recoil/batteryState';

function SelectCarTime({ setIsReady }) {
  const [chargeTime, setChargeTime] = useState('');
  const [carData, setCarData] = useState([]);
  const [currentIndex, setCurrentIndex] = useRecoilState(currentIndexState); // Recoil 상태로 currentIndex 가져오기
  const accessToken = useRecoilValue(accessTokenState); 
  const setParkingState = useSetRecoilState(parkingState);
  const setBatteryState = useSetRecoilState(batteryState);  

  useEffect(() => {
    const fetchCarData = async () => {
      try {
        const response = await axios.get('https://j11c208.p.ssafy.io/api/v1/cars/me', {
          headers: { Authorization: `Bearer ${accessToken}` },
        });
        
        console.log('API Response:', response.data); 
        
        if (response.data && response.data.data && response.data.data.length > 0) {
          setCarData(response.data.data);
        } else {
          console.error('차량 정보가 없습니다.');
          setCarData([]);
        }
      } catch (error) {
        console.error('Error fetching car data:', error.message); 
      }
    };

    fetchCarData();
  }, [accessToken]);

  const handlers = useSwipeable({
    onSwipedLeft: () => setCurrentIndex((prevIndex) => (prevIndex + 1) % carData.length),
    onSwipedRight: () => setCurrentIndex((prevIndex) => (prevIndex - 1 + carData.length) % carData.length),
    preventDefaultTouchmoveEvent: true,
    trackMouse: true,
  });

  if (carData.length === 0) {
    return <div>차량 정보를 로딩 중입니다...</div>; 
  }

  const handleChargeTimeChange = (e) => {
    const selectedValue = e.target.value;
    setChargeTime(selectedValue);

    const isFullCharge = selectedValue === 'perfect';
    const timeValue = selectedValue !== '' ? Number(selectedValue) : 0;

    setParkingState((prevState) => ({
      ...prevState,
      carId: carData[currentIndex].id,
      battery: carData[currentIndex].battery,
      fullCharge: isFullCharge,
      time: timeValue,
    }));

    setBatteryState(carData[currentIndex].battery);

    console.log('Updated Parking State:', {
      carId: carData[currentIndex].id,
      fullCharge: isFullCharge,
      time: timeValue,
    });

    if (selectedValue) {
      setIsReady(true);
    } else {
      setIsReady(false);
    }
  };

  return (
    <div className='SelectCarTime' {...handlers}>
      <div className='SelectCarTime_contents_box'>
        <div className='SelectCarTime_contents1'>차량 및 충전시간을</div>
        <div className='SelectCarTime_contents2'>선택해 주세요.</div>

        <div className='SelectCar'>
          <img 
            src={carData[currentIndex].carType.image} 
            alt={carData[currentIndex].carType.name} 
            className='SelectCarTime_Carimg' 
          />
          <div className='SelectCarTime_contents'>
            <div>
              <div className='CarType'>
                <div className='Car_MonthInformation1'>차량 종류</div>
                <div className='MonthPayment_Contents2'>{carData[currentIndex].carType.name}</div>
              </div>
            </div>
            <div>
              <div className='Car_MonthInformation1'>배터리</div>
              <div className='MonthPayment_Contents'>
                <div>
                  {[...Array(10)].map((_, index) => {
                    const batteryLevel = carData[currentIndex]?.battery || 0;
                    let batteryClass = 'NoChargeBattery_img';

                    if (batteryLevel === 100) {
                      batteryClass = 'ChargeBattery_img1';
                    } else if (batteryLevel >= 70) {
                      batteryClass = 'ChargeBattery_img2';
                    } else if (batteryLevel >= 30) {
                      batteryClass = 'ChargeBattery_img3';
                    } else if (batteryLevel > 0) {
                      batteryClass = 'ChargeBattery_img4';
                    }

                    return (
                      <div 
                        key={index} 
                        className={batteryLevel >= (10 - index) * 10 
                          ? batteryClass 
                          : 'NoChargeBattery_img'}
                      ></div>
                    );
                  })}
                </div>
                <div>{carData[currentIndex]?.battery || 0}</div>
                <div className='small_font'>%</div>
              </div>
            </div>

            <div>
              <div className='Car_MonthInformation1'>차량 번호</div>
              <div className='MonthPayment_Contents'>
                <div className='small_font'>{carData[currentIndex]?.number?.slice(0, 3) || "N/A"}</div>
                <div>{carData[currentIndex]?.number?.slice(3) || "N/A"}</div>
              </div>
            </div>

          </div>
        </div>

        <p></p>
        <div className='contents'>충전 시간 설정</div>
        <select 
          value={chargeTime} 
          onChange={handleChargeTimeChange} 
          className='SelectTime'
        >
          <option value="">충전 시간을 선택하세요</option>
          <option value="30">30분</option>
          <option value="60">1시간</option>
          <option value="90">1시간 30분</option>
          <option value="120">2시간</option>
          <option value="perfect">완전 충전</option>
        </select>
      </div>
    </div>
  );
}

export default SelectCarTime;
