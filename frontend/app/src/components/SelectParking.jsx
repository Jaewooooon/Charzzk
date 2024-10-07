import React, { useState, useEffect } from 'react';
import axios from 'axios';
import { useRecoilState } from 'recoil';
import { parkingState } from '../recoil/parkingState'; // parkingState atom import
import '../components/styles/SelectParking.css'; // CSS 파일 임포트

function SelectParking({ setIsReady }) { // setIsReady를 props로 추가
  const [parkingLots, setParkingLots] = useState([]); // 주차장 목록 상태
  const [searchTerm, setSearchTerm] = useState(''); // 검색어 상태
  const [location, setLocation] = useState({ latitude: null, longitude: null }); // 위치 상태
  const [error, setError] = useState(null); // 에러 상태
  const [selectedParkingLotIndex, setSelectedParkingLotIndex] = useState(null); // 선택된 주차장 인덱스 상태
  const [parkingData, setParkingData] = useRecoilState(parkingState); // Recoil 상태 사용

  useEffect(() => {
    const getCurrentLocation = () => {
      if (navigator.geolocation) {
        navigator.geolocation.getCurrentPosition(
          (position) => {
            const { latitude, longitude } = position.coords;
            setLocation({ latitude, longitude });
            fetchParkingLots(latitude, longitude);
          },
          (error) => {
            setError("위치를 가져오는 데 오류가 발생했습니다: " + error.message);
          }
        );
      } else {
        setError("이 브라우저는 Geolocation을 지원하지 않습니다.");
      }
    };

    const fetchParkingLots = (latitude, longitude) => {
      axios.get('https://j11c208.p.ssafy.io/api/v1/parking-lot', {
        params: {
          latitude: latitude,
          longitude: longitude
        }
      })
      .then(response => {
        setParkingLots(response.data.data); // 주차장 리스트 상태 업데이트
      })
      .catch(error => {
        console.error("주차장 가져오기 오류:", error);
      });
    };

    getCurrentLocation();
  }, []); // 빈 배열을 전달하여 컴포넌트가 마운트될 때만 실행

  const filteredParkingLots = parkingLots.filter(lot =>
    lot.name.toLowerCase().includes(searchTerm.toLowerCase())
  );

  const handleParkingLotSelect = (index) => {
    if (selectedParkingLotIndex === index) {
      setSelectedParkingLotIndex(null);
      setIsReady(false); // 버튼 상태 false로 변경
      setParkingData((prevData) => ({ ...prevData, parkingLotId: null })); // parkingLotId 초기화
    } else {
      setSelectedParkingLotIndex(index);
      setIsReady(true); // 버튼 상태 true로 변경
      setParkingData((prevData) => ({ ...prevData, parkingLotId: parkingLots[index].id })); // parkingLotId 업데이트
    }
  };

  return (
    <div className='SelectParking'>
      <div className='SelectParking_contents_box'>
        <div className='SelectParking_contents1'>주차장을</div>
        <div className='SelectParking_contents2'>선택해 주세요.</div>
      </div>

      <input
        type="text"
        placeholder="주차장 검색..."
        className='SearchInput'
        value={searchTerm}
        onChange={(e) => setSearchTerm(e.target.value)} // 검색어 상태 업데이트
      />

      <ul className='ParkingList'>
        {filteredParkingLots.length > 0 ? (
          filteredParkingLots.map((lot, index) => (
            <li
              key={index}
              className='ParkingItem'
              onClick={() => handleParkingLotSelect(index)} // 주차장 선택 핸들러 호출
              style={{
                backgroundColor: selectedParkingLotIndex === index ? 'lightgray' : 'transparent', // 선택된 주차장 배경색 변경
              }}
            >
              <img src={lot.image} alt="주차장 이미지" className='ParkingImage' />
              <div className='ParkingInfo'>
                <h3 className='ParkingName'>{lot.name}</h3>
                <div className='ParkingAddress'>{lot.address}</div>
                <div className='ParkingDistance'>{Math.floor(lot.distance)}m</div>
              </div>
            </li>
          ))
        ) : (
          <li className='NoResults'>일치하는 주차장이 없습니다</li>
        )}
      </ul>
      {error && <div className='Error'>{error}</div>} {/* 에러 메시지 표시 */}
    </div>
  );
}

export default SelectParking;
