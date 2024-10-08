import React, { useState, useEffect } from 'react';
import '../components/styles/StopChargingButton.css';
import DangerConfirm from '../assets/DangerConfirm.png';
import { currentIndexState } from '../recoil/CurrentIndex';
import { accessTokenState } from '../recoil/LoginAtom';
import { useRecoilState, useRecoilValue } from 'recoil';  // useRecoilValue 추가
import axios from 'axios';

function StopChargingButton() {
  const [isModalOpen, setIsModalOpen] = useState(false); // 모달창 열기/닫기 상태
  const [carData, setCarData] = useState([]);  // 차량 데이터를 저장할 상태
  const [payment, setPayment] = useState(0);  // 충전 요금을 저장할 상태
  const [chargeAmount, setChargeAmount] = useState(0);  // 충전량을 저장할 상태
  const [currentIndex, setCurrentIndex] = useRecoilState(currentIndexState);
  const accessToken = useRecoilValue(accessTokenState);  // useRecoilValue로 accessToken 가져오기

  // 차량 데이터 가져오는 함수
  const fetchCarData = async () => {
    try {
      const response = await axios.get('https://j11c208.p.ssafy.io/api/v1/cars/me', {
        headers: {
          Authorization: `Bearer ${accessToken}`,
        },
      });
                                                                                                                                                                                                                                                                                                                                         
      if (response.data.code === 200) {
        setCarData(response.data.data); // 차량 데이터를 상태에 저장
        if (response.data.data.length > 0) {
          setPayment(response.data.data[currentIndex].chargeCost || 0); // 현재 차량의 충전 요금을 설정
          setChargeAmount(response.data.data[currentIndex].chargeAmount || 0); // 현재 차량의 충전량을 설정
        }
      } else {
        console.error('Error fetching car data:', response.data.message);
      }
    } catch (error) {
      console.error('Error fetching data:', error);
    }
  };

  // 컴포넌트 마운트 시 차량 데이터 가져오기
  useEffect(() => {
    fetchCarData();
  }, [currentIndex, accessToken]);  // currentIndex와 accessToken이 변경될 때마다 데이터를 다시 가져오기


  const handleStopChargingClick = () => {
    setIsModalOpen(true); // 버튼 클릭 시 모달창 열기
  };

  const confirmStopCharging = async () => {
    const reservationId = carData[currentIndex].reservationId;
    try {
      await axios.delete(`https://j11c208.p.ssafy.io/api/v1/reservations/${reservationId}`, {
        headers: {
          Authorization: `Bearer ${accessToken}`,
        },
      });
      alert('충전이 중단되었습니다.');
    } catch (error) {
      console.error(error);
      alert('충전 중단에 실패했습니다.');
    }
    setIsModalOpen(false); // 모달창 닫기
  };

  const cancelStopCharging = () => {
    setIsModalOpen(false); // 모달창 닫기
  };

  return (
    <div>
      
      <button className='StopChargingButton' onClick={handleStopChargingClick}>
        중단하기
      </button>
      

      {/* 모달창 */}
      {isModalOpen && (
        <div className="modal">
          <div className="modal-content">
            <img src={DangerConfirm} alt="dangerlogo" />
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
