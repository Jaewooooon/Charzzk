import React, { useState, useEffect } from 'react';
import '../styles/ChargeStatus.css';
import GoBackButton from '../components/GobackButton';
import StopChargingButton from '../components/StopChargingButton';
import { currentIndexState } from '../recoil/CurrentIndex';
import { accessTokenState } from '../recoil/LoginAtom';
import { useRecoilState, useRecoilValue } from 'recoil';
import axios from 'axios';

const ChargeStatus = () => {
  const [carData, setCarData] = useState([]);  
  const [reservationData, setReservationData] = useState(null);  // 예약 데이터 상태 추가
  const [payment, setPayment] = useState(0);  
  const [chargeAmount, setChargeAmount] = useState(0);  
  const [currentIndex, setCurrentIndex] = useRecoilState(currentIndexState);
  const accessToken = useRecoilValue(accessTokenState); 

  // 차량 데이터 가져오는 함수
  const fetchCarData = async () => {
    try {
      const response = await axios.get('https://j11c208.p.ssafy.io/api/v1/cars/me', {
        headers: {
          Authorization: `Bearer ${accessToken}`,
        },
      });

      if (response.data.code === 200) {
        setCarData(response.data.data); 
        if (response.data.data.length > 0) {
          setPayment(response.data.data[currentIndex].chargeCost || 0); 
          setChargeAmount(response.data.data[currentIndex].chargeAmount || 0); 
          fetchReservationData();  // 선택된 차량 ID로 예약 정보 요청
        }
      } else {
        console.error('Error fetching car data:', response.data.message);
      }
    } catch (error) {
      console.error('Error fetching data:', error);
    }
  };

  // 예약 데이터를 가져오는 함수
  const fetchReservationData = async () => {
    const carId = carData[currentIndex].id;  // currentIndex에 따라 차량 ID 가져오기
    try {
      const response = await axios.get(`https://j11c208.p.ssafy.io/api/v1/cars/${carId}`, {
        headers: {
          Authorization: `Bearer ${accessToken}`,
        },
      });

      if (response.data.code === 200) {
        console.log('Reservation Data Response:', response.data);
        setReservationData(response.data.data);  // 예약 데이터 상태에 저장
      } else {
        console.error('Error fetching reservation data:', response.data.message);
      }
    } catch (error) {
      console.error('Error fetching reservation data:', error);
    }
  };

  // 데이터 주기적으로 업데이트
  useEffect(() => {
    fetchCarData();  // 처음 컴포넌트가 마운트되었을 때 데이터 요청

    const intervalId = setInterval(() => {
      fetchCarData();  // 일정 시간마다 데이터를 요청하여 갱신
    }, 1000); // 1초마다 데이터를 요청

    return () => clearInterval(intervalId);  // 컴포넌트가 언마운트되면 interval을 정리
  }, [currentIndex, accessToken]);

  // 배터리 상태에 따른 스타일 계산 (2%씩 차이를 줌)
  const calculateHeight = (battery, offset) => {
    return `${Math.max(battery - offset, 0)}%`;  // 최소 0%로 설정
  };

  return (
    <div className="ChargeStatus_ContainerBox">
      <GoBackButton />

      <div className="Percent_Contents">
        <p className='AutoCharge_Contents'> AutoCharge!</p>
        <p className='ChargePercent_Contents'>{carData.length > 0 && currentIndex < carData.length && `${carData[currentIndex].battery}%`}</p>

        {/* 예약 데이터가 있을 때 표시 */}
        {reservationData && (
          <>
            <p className='ChargeStatus_Contents'>충전 상태: {reservationData.status}</p>
            <p className='ChargeStart_Contents'>충전 시작 시각: {new Date(reservationData.startTime).toLocaleTimeString()}</p>
            <p className='ChargeComplete_Contents'>충전 완료 시각: {new Date(reservationData.endTime).toLocaleTimeString()}</p>
          </>
        )}
      </div>

      {/* 각 요소의 높이를 동적으로 설정 (2% 차이씩) */}
      <div 
        className='Percent_Show1' 
        style={{ height: calculateHeight(carData.length > 0 && currentIndex < carData.length ? carData[currentIndex].battery : 0, 0) }}
      ></div>
      <div 
        className='Percent_Show2' 
        style={{ height: calculateHeight(carData.length > 0 && currentIndex < carData.length ? carData[currentIndex].battery : 0, 1) }}
      ></div>
      <div 
        className='Percent_Show3' 
        style={{ height: calculateHeight(carData.length > 0 && currentIndex < carData.length ? carData[currentIndex].battery : 0, -1) }}
      ></div>
      <div 
        className='Percent_Show4' 
        style={{ height: calculateHeight(carData.length > 0 && currentIndex < carData.length ? carData[currentIndex].battery : 0, 0) }}
      ></div>
      <div 
        className='Percent_Show5' 
        style={{ height: calculateHeight(carData.length > 0 && currentIndex < carData.length ? carData[currentIndex].battery : 0, 1) }}
      ></div>

      <StopChargingButton />
    </div>
  );
};

export default ChargeStatus;
