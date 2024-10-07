import React, { useState, useEffect } from 'react';
import axios from 'axios';
import { useRecoilValue, useSetRecoilState } from 'recoil'; 
import { accessTokenState } from '../recoil/LoginAtom'; 
import { parkingState } from '../recoil/parkingState'; 
import '../components/styles/SelectCarTime.css'; 
import { useSwipeable } from 'react-swipeable';

function SelectCarTime({ setIsReady }) {
  const [chargeTime, setChargeTime] = useState('');
  const [carData, setCarData] = useState([]);
  const [currentIndex, setCurrentIndex] = useState(0);
  const accessToken = useRecoilValue(accessTokenState); 
  const setParkingState = useSetRecoilState(parkingState); 

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

    // carData[currentIndex].id를 사용하여 carId를 설정
    setParkingState((prevState) => ({
      ...prevState,
      carId: carData[currentIndex].id, // 차량의 id를 설정
      fullCharge: isFullCharge,
      time: timeValue,
    }));

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
            <div>차량 번호: {carData[currentIndex].number}</div>
            <div>차량 종류: {carData[currentIndex].carType.name}</div>
            <div>배터리 잔량: {carData[currentIndex].battery}%</div>
          </div>
        </div>
        
        
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
