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
import CompleteChargeModal from '../components/CompleteChargeModal';
import StepDownButton from '../components/StepDownButton'; // StepDownButton 임포트
import { batteryState } from '../recoil/batteryState';
import { buttonState } from '../recoil/buttonState';
import { parkingState } from '../recoil/parkingState'; 
import { accessTokenState } from '../recoil/LoginAtom';
import { useNavigate } from 'react-router-dom';

const ChargeStart = () => {
  const [isReady, setIsReady] = useRecoilState(buttonState);
  const [step, setStep] = useState(0);
  const [reservationData, setReservationData] = useState(null); 
  const [isLoading, setIsLoading] = useState(false); 
  const [isModalOpen, setIsModalOpen] = useState(false); // 모달 열기 상태
  const parkingInfo = useRecoilValue(parkingState);
  const accessToken = useRecoilValue(accessTokenState);
  const batteryValue = useRecoilValue(batteryState);
  const navigate = useNavigate();

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

  const goToPreviousStep = () => {
    if (step > 0) {
      setStep(prevStep => prevStep - 1);
    }
  };

  const handleReservation = async () => {
    setIsLoading(true); 
    const requestData = {
      parkingSpotId: parkingInfo.parkingSpotId,
      carId: parkingInfo.carId,
      parkingLotId: parkingInfo.parkingLotId,
      fullCharge: parkingInfo.fullCharge,
      time: parkingInfo.time,
      battery: parkingInfo.battery,
    };

    console.log('예약 요청 데이터:', requestData); 
    console.log('전송할 Authorization 헤더:', `Bearer ${accessToken}`);

    try {
      const response = await axios.post(
        'https://j11c208.p.ssafy.io/api/v1/reservations',
        requestData,  
        {
          headers: {
            Authorization: `Bearer ${accessToken}`, 
          },
        }
      );

      setReservationData(response.data); 
      setIsModalOpen(true); // 예약 완료 후 모달 열기
    } catch (error) {
      console.error('예약 요청 중 오류 발생:', error.response?.data || error.message);
    } finally {
      setIsLoading(false); 
    }
  };

  const startCharging = async () => {
    const reservationId = reservationData.data.id; // 예약 ID 가져오기
    setIsLoading(true); // 로딩 시작
    console.log(reservationId);

    try {
      const response = await axios.patch(
        `https://j11c208.p.ssafy.io/api/v1/reservations/${reservationId}`, // 예약 ID 포함
        null,
        {
          headers: {
            Authorization: `Bearer ${accessToken}`, // Authorization 헤더에 accessToken 추가
          },
        }
      );

      console.log('충전 시작 응답:', response.data); // 응답 데이터 확인
      // 충전 시작 후 필요한 추가 로직 구현
      navigate('/charge-status');

    } catch (error) {
      console.error('충전 시작 요청 중 오류 발생:', error.response?.data || error.message);
    } finally {
      setIsLoading(false); // 로딩 종료
    }
  };

  useEffect(() => {
    if (step === 3) {
      handleReservation(); 
    }
  }, [step]);

  const closeModal = () => {
    setIsModalOpen(false); // 모달 닫기
  };

  const handleCancel = () => {
    setIsModalOpen(false); // 모달 닫기
    setStep(2); // Step을 2로 변경
  };

  // 모달이 열리고 30초간 아무 이벤트가 없으면 Step을 2로 변경하는 타이머 추가
  useEffect(() => {
    let timer;
    if (isModalOpen) {
      timer = setTimeout(() => {
        setIsModalOpen(false); // 모달 닫기
        setStep(2); // Step을 2로 변경
      }, 30000); // 30초 후
    }

    // 컴포넌트가 언마운트되거나 모달이 닫힐 때 타이머를 정리
    return () => clearTimeout(timer);
  }, [isModalOpen]);

  const calculateMinutesUntilStart = (startTime) => {
    const now = new Date();
    const startDateTime = new Date(startTime);
    const diffInMs = startDateTime - now; // 밀리초 차이 계산
    const diffInMinutes = Math.ceil(diffInMs / (1000 * 60)); // 분으로 변환

    return diffInMinutes >= 0 ? diffInMinutes : 0; // 음수일 경우 0으로 처리
  };

  const minutesUntilStart = reservationData ? calculateMinutesUntilStart(reservationData.data.startTime) : null;

  return (
    <div>
      <GoBackButton />
      {isReady ? <ReadyNextButton onClick={goToNextStep} /> : <NotReadyNextButton />}
      
      <StepDownButton onClick={goToPreviousStep} /> {/* 이전 단계 버튼 추가 */}

      {step === 0 && <SelectParking setIsReady={setIsReady} />}
      {step === 1 && <SelectParking2 setIsReady={setIsReady} />}
      {step === 2 && <SelectCarTime setIsReady={setIsReady} />}
      {step === 3 && <SelectCarTime setIsReady={setIsReady} />}

      {/* 모달 창을 통해 예약 정보 표시 */}
      <CompleteChargeModal isOpen={isModalOpen} onClose={closeModal}>
        {isLoading ? (
          <p>로딩 중...</p>
        ) : reservationData ? (
          <div className='CompleteCharge_contents'>
            <p className='battery_value'>{batteryValue}%</p>
            <div className='HowWaiting'>{minutesUntilStart !== null ? 
              `${minutesUntilStart === 0 ? "바로 충전 시작" : `${minutesUntilStart}분 후 충전 시작`}` 
              : "충전 시작 시간을 가져오는 데 실패했습니다."}</div>
            <p className='chargestart_content'>충전시작  {new Date(reservationData.data.startTime).toLocaleString()}</p>
            <p className='chargeend_content'>충전완료  {new Date(reservationData.data.endTime).toLocaleString()}</p>
            <div >
            <button className='ChargeStart_btn' onClick={startCharging} >충전 시작</button>
            <button className='Cancel_btn' onClick={handleCancel}>취소하기</button>
            </div>
            

          </div>
        ) : (
          <p>예약 정보를 가져오는 데 실패했습니다.</p>
        )}
      </CompleteChargeModal>
    </div>
  );
};

export default ChargeStart;
