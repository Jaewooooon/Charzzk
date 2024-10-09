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
  const [reservationData, setReservationData] = useState(null); 
  const [payment, setPayment] = useState(0);  
  const [chargeAmount, setChargeAmount] = useState(0);  
  const [currentIndex, setCurrentIndex] = useRecoilState(currentIndexState);
  const accessToken = useRecoilValue(accessTokenState); 

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
          const selectedCar = response.data.data[currentIndex];
          setPayment(selectedCar.chargeCost || 0); 
          setChargeAmount(selectedCar.chargeAmount || 0); 
          fetchReservationData(selectedCar.id); 
        }
      } else {
        console.error('Error fetching car data:', response.data.message);
      }
    } catch (error) {
      console.error('Error fetching data:', error);
    }
  };

  const fetchReservationData = async (carId) => {
    try {
      const response = await axios.get(`https://j11c208.p.ssafy.io/api/v1/cars/${carId}`, {
        headers: {
          Authorization: `Bearer ${accessToken}`,
        },
      });

      if (response.data.code === 200) {
        console.log('Reservation Data Response:', response.data);
        setReservationData(response.data.data);  
      } else {
        console.error('Error fetching reservation data:', response.data.message);
      }
    } catch (error) {
      console.error('Error fetching reservation data:', error);
    }
  };

  useEffect(() => {
    fetchCarData(); 

    const intervalId = setInterval(() => {
      fetchCarData(); 
    }, 1000);

    return () => clearInterval(intervalId); 
  }, [currentIndex, accessToken]); 

  const calculateHeight = (battery, offset) => {
    return `${Math.max(battery - offset, 0)}%`; 
  };

  const calculateRemainingTime = (endTime) => {
    const now = new Date();
    const end = new Date(endTime);
    const remainingTime = Math.floor((end - now) / 60000); 
    return remainingTime > 0 ? remainingTime : 0; 
  };

  return (
    <div className="ChargeStatus_ContainerBox">
      <GoBackButton />

      <div className="Percent_Contents">
        <p className='AutoCharge_Contents'>{reservationData ? reservationData.status : '예약 정보 없음'}</p>
        <p className='ChargePercent_Contents'>{carData.length > 0 && currentIndex < carData.length && `${carData[currentIndex].battery}%`}</p>

        {reservationData && (
          <>
            {reservationData.status === 'WAITING' && (
              <>
                <p className='ChargeStatus_Contents'>{calculateRemainingTime(reservationData.startTime)} 분 후 충전 시작</p>
                <p className='ChargeStart_Contents'>충전 시작 <div className='charge_time'> {new Date(reservationData.startTime).toLocaleTimeString()}</div> 예정</p>
                <p className='ChargeComplete_Contents'>충전 완료 <div className='charge_time'> {new Date(reservationData.endTime).toLocaleTimeString()} </div>예정</p>
              </>
            )}

            {reservationData.status === 'CHARGING' && (
              <>
                <p className='ChargeStatus_Contents'>{calculateRemainingTime(reservationData.endTime)} 분 후 충전 완료</p>
                <p className='ChargeStart_Contents'>충전 시작 <div className='charge_time'> {new Date(reservationData.startTime).toLocaleTimeString()}</div> 시작</p>
                <p className='ChargeComplete_Contents'>충전 완료 <div className='charge_time'> {new Date(reservationData.endTime).toLocaleTimeString()} </div>예정</p>
              </>
            )}

            {reservationData.status === 'DONE' && (
              <>
                <p className='ChargeComplete_Contents'>충전 완료 <div className='charge_time'> {new Date(reservationData.endTime).toLocaleTimeString()}</div> 완료</p>
              </>
            )}
          </>
        )}
      </div>

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

      {/* Conditional rendering of StopChargingButton */}
      {(reservationData && (reservationData.status === 'CHARGING' || reservationData.status === 'WAITING')) && (
        <StopChargingButton />
      )}
    </div>
  );
};

export default ChargeStatus;
