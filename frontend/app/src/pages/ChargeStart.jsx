import React, { useEffect, useState } from 'react';
import { useRecoilState, useRecoilValue } from 'recoil';
import axios from 'axios';
import '../styles/Charge_Start.css';
import GoBackButton from '../components/GobackButton';
import NotReadyNextButton from '../components/NotReadyNextButton';
import ReadyNextButton from '../components/ReadyNextButton';
import SelectParking from '../components/SelectParking';
import SelectParking2 from '../components/SelectParking2';
import SelectCarTime from '../components/SelectCarTime';
import { buttonState } from '../recoil/buttonState'; 
import { parkingState } from '../recoil/parkingState'; // parkingState atom import
import { accessTokenState } from '../recoil/LoginAtom'; // accessTokenState import

const ChargeStart = () => {
  const [isReady, setIsReady] = useRecoilState(buttonState);
  const [step, setStep] = useState(0);
  const [reservationData, setReservationData] = useState(null); // 응답 데이터를 저장할 상태
  const [isLoading, setIsLoading] = useState(false); // 로딩 상태
  const parkingInfo = useRecoilValue(parkingState); // Recoil에서 주차 정보 가져오기
  const accessToken = useRecoilValue(accessTokenState); // Recoil에서 로그인 토큰 가져오기

  useEffect(() => {
    return () => {
      setIsReady(false);
    };
  }, [setIsReady]);

  const goToNextStep = () => {
    if (isReady) {
      setStep(prevStep => prevStep + 1);
      setIsReady(false);
    }
  };

  // API 요청 함수
  const handleReservation = async () => {
    setIsLoading(true); // 로딩 시작
    const requestData = {
      parkingSpotId: parkingInfo.parkingSpotId,
      carId: parkingInfo.carId,
      parkingLotId: parkingInfo.parkingLotId,
      fullCharge: parkingInfo.fullCharge,
      time: parkingInfo.time,
    };
  
    console.log('예약 요청 데이터:', requestData); // requestData를 콘솔에 출력
  
    console.log('전송할 Authorization 헤더:', `Bearer ${accessToken}`);
  
    try {
      const response = await axios.post(
        'https://j11c208.p.ssafy.io/api/v1/reservations',
        requestData,  // 전송할 데이터를 requestData로 통일
        {
          headers: {
            Authorization: `Bearer ${accessToken}`, // 헤더에 토큰 담기
          },
        }
      );
  
      setReservationData(response.data); // 응답 데이터 저장
    } catch (error) {
      console.error('예약 요청 중 오류 발생:', error.response?.data || error.message);
    } finally {
      setIsLoading(false); // 로딩 종료
    }
  };
  

  // step이 3일 때 예약 요청 및 배경 처리
  useEffect(() => {
    if (step === 3) {
      handleReservation(); // 예약 요청
    } else {
      document.body.style.backgroundColor = ''; // 기본 배경색으로 복원
    }
  }, [step]);

  return (
    <div>
      <GoBackButton />
      {isReady ? <ReadyNextButton onClick={goToNextStep} /> : <NotReadyNextButton />}
      
      {step === 0 && <SelectParking setIsReady={setIsReady} />}
      {step === 1 && <SelectParking2 setIsReady={setIsReady} />}
      {step === 2 && <SelectCarTime setIsReady={setIsReady} />}
      {step === 3 && (
        <div>
          {isLoading ? (
            <p>로딩 중...</p> // 로딩 상태 표시
          ) : reservationData ? (
            <div>
              <h2>예약 정보</h2>
              <p>차량: {reservationData.data.car.nickname} ({reservationData.data.car.number})</p>
              <p>시작 시간: {new Date(reservationData.data.startTime).toLocaleString()}</p>
              <p>종료 시간: {new Date(reservationData.data.endTime).toLocaleString()}</p>
              <p>상태: {reservationData.data.status}</p>
            </div>
          ) : (
            <p>예약 정보를 가져오는 데 실패했습니다.</p>
          )}
        </div>
      )}
    </div>
  );
}

export default ChargeStart;
