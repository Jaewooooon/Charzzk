import React, { useEffect, useState } from 'react';
import { useRecoilValue, useSetRecoilState } from 'recoil';
import axios from 'axios';
import { parkingState } from '../recoil/parkingState.jsx';
import '../components/styles/SelectParking2.css';
import ChargeMap from '../assets/ChargeMap.png';

function SelectParking2({ setIsReady }) {
  const [parkingSpots, setParkingSpots] = useState([]); // 주차 공간 상태
  const [selectedSpot, setSelectedSpot] = useState(''); // 선택된 주차 공간 상태
  const currentParkingState = useRecoilValue(parkingState); // Recoil에서 현재 상태 가져오기
  const setParkingState = useSetRecoilState(parkingState); // 상태 업데이트 함수

  useEffect(() => {
    if (currentParkingState.parkingLotId) {
      // parkingLotId가 있을 때 API 호출
      axios.get(`https://j11c208.p.ssafy.io/api/v1/parking-lot/${currentParkingState.parkingLotId}`)
        .then(response => {
          if (response.data.code === 200) {
            setParkingSpots(response.data.data.parkingSpots); // 주차 공간 데이터 설정
          }
        })
        .catch(error => {
          console.error("주차 공간 데이터 가져오기 오류:", error);
        });
    }
  }, [currentParkingState.parkingLotId]);

  const handleSpotSelect = (event) => {
    const selectedName = event.target.value;
    setSelectedSpot(selectedName);

    const selectedSpotObj = parkingSpots.find(spot => spot.name === selectedName); // 선택된 주차 공간 객체 찾기
    if (selectedSpotObj) {
      setParkingState(prevState => ({
        ...prevState,
        parkingSpotId: selectedSpotObj.id, // 선택한 주차 공간의 id 저장
      }));
      setIsReady(true); // 주차 공간 선택 여부에 따라 버튼 상태 업데이트
    } else {
      setIsReady(false);
    }
  };

  // Recoil 상태 변경될 때마다 콘솔에 출력
  useEffect(() => {
    console.log('Recoil parkingState:', currentParkingState);
  }, [currentParkingState]); // currentParkingState가 변경될 때마다 실행

  return (
    <div className='SelectParking2'>
      <div className='SelectParking2_contents_box'>
        <div className='SelectParking2_contents1'>주차 위치를</div>
        <div className='SelectParking2_contents2'>선택해 주세요.</div>
        <img src={ChargeMap} alt="충전지도" className='ChargeMapimg' />
      </div>

      <div className='contents'>주차 위치 입력</div>
      <select value={selectedSpot} onChange={handleSpotSelect} className='ParkingSpotSelect'>
        <option value="">주차 공간을 선택하세요</option>
        {parkingSpots.map(spot => (
          <option key={spot.id} value ={spot.name}>{spot.name}</option>
        ))}
      </select>
    </div>
  );
}

export default SelectParking2;
