import React, { useState, useEffect } from 'react';
import '../components/styles/StopChargingButton.css';
import DangerConfirm from '../assets/DangerConfirm.png';
import { currentIndexState } from '../recoil/CurrentIndex';
import { accessTokenState } from '../recoil/LoginAtom';
import { useRecoilState, useRecoilValue } from 'recoil'; 
import axios from 'axios';

function StopChargingButton() {
  const [isModalOpen, setIsModalOpen] = useState(false); 
  const [carData, setCarData] = useState([]); 
  const [payment, setPayment] = useState(0);  
  const [chargeAmount, setChargeAmount] = useState(0); 
  const [reservationId, setReservationId] = useState(null); // reservationId 상태 추가
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
          await fetchReservationId(selectedCar.id); // 선택된 차량의 id로 예약 ID를 가져오는 함수 호출
        }
      } else {
        console.error('Error fetching car data:', response.data.message);
      }
    } catch (error) {
      console.error('Error fetching data:', error);
    }
  };

  // 예약 ID를 가져오는 함수
  const fetchReservationId = async (carId) => {
    try {
      const response = await axios.get(`https://j11c208.p.ssafy.io/api/v1/cars/${carId}`, {
        headers: {
          Authorization: `Bearer ${accessToken}`,
        },
      });

      if (response.data.code === 200) {
        setReservationId(response.data.data.reservationId); // reservationId 상태 설정
      } else {
        console.error('Error fetching reservation ID:', response.data.message);
      }
    } catch (error) {
      console.error('Error fetching reservation ID:', error);
    }
  };

  useEffect(() => {
    fetchCarData(); 
  }, [currentIndex, accessToken]);

  const handleStopChargingClick = () => {
    setIsModalOpen(true);
  };

  const confirmStopCharging = async () => {
    try {
      if (reservationId) { // reservationId가 있는지 확인
        await axios.delete(`https://j11c208.p.ssafy.io/api/v1/reservations/${reservationId}`, {
          headers: {
            Authorization: `Bearer ${accessToken}`, // 헤더에 토큰 추가
          },
        });
        alert('충전이 중단되었습니다.');
      } else {
        alert('예약 ID가 없습니다. 충전 중단에 실패했습니다.');
      }
    } catch (error) {
      console.error(error);
      alert('충전 중단에 실패했습니다.');
    }
    setIsModalOpen(false); 
  };

  const cancelStopCharging = () => {
    setIsModalOpen(false);
  };

  return (
    <div>
      <button className='StopChargingButton' onClick={handleStopChargingClick}>
        중단하기
      </button>

      {isModalOpen && (
        <div className="stop-modal">
          <div className="stop-modal-content">
            <img src={DangerConfirm} alt="dangerlogo" className='dangerlogo'/>
            <div className='ReallyStop'>정말 중단하시겠습니까?</div>
            <div>
              <button onClick={confirmStopCharging} className='confirmStopCharging_btn'>확인</button>
              <button onClick={cancelStopCharging} className='cancelStopCharging_btn'>취소</button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}

export default StopChargingButton;
