import React, { useState, useEffect } from 'react';
import axios from 'axios';
import { useRecoilState } from 'recoil';
import { parkingState } from '../recoil/parkingState.jsx'; // parkingState atom import
import '../components/styles/SelectParking.css'; // CSS 파일 임포트

function SelectParking({ setIsReady }) {
  const [parkingLots, setParkingLots] = useState([]); // 주차장 목록 상태
  const [searchTerm, setSearchTerm] = useState(''); // 검색어 상태
  const [location, setLocation] = useState({ latitude: null, longitude: null }); // 위치 상태
  const [error, setError] = useState(null); // 에러 상태
  const [selectedParkingLotIndex, setSelectedParkingLotIndex] = useState(null); // 선택된 주차장 인덱스 상태
  const [parkingData, setParkingData] = useRecoilState(parkingState); // Recoil 상태 사용
  const [sortOrder, setSortOrder] = useState('distance'); // 초기 정렬 기준
  const [activeButton, setActiveButton] = useState('distance'); // 선택된 버튼 상태

  useEffect(() => {
    const getCurrentLocation = () => {
      if (navigator.geolocation) {
        navigator.geolocation.getCurrentPosition(
          (position) => {
            const { latitude, longitude } = position.coords;
            setLocation({ latitude, longitude });
            fetchParkingLots(latitude, longitude, sortOrder);
          },
          (error) => {
            setError("위치를 가져오는 데 오류가 발생했습니다: " + error.message);
          }
        );
      } else {
        setError("이 브라우저는 Geolocation을 지원하지 않습니다.");
      }
    };

    const fetchParkingLots = (latitude, longitude, sort) => {
      axios.get('https://j11c208.p.ssafy.io/api/v1/parking-lot', {
        params: {
          latitude: latitude,
          longitude: longitude,
          sort: sort // 정렬 기준 추가
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
  }, [sortOrder]); // sortOrder가 변경될 때마다 실행

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

  const handleSort = (order) => {
    setSortOrder(order); // 정렬 기준 업데이트
    setActiveButton(order); // 활성화된 버튼 상태 업데이트
  };

  const formatWaitingTime = (waitingTime) => {
    if (waitingTime === 0) {
      return <span className='ReadyNow'>지금 바로 충전 가능</span>;
    } else if (waitingTime > 0 && waitingTime <= 60) {
      return <span className='NotReadyNow'>{waitingTime}분 후 충전 가능</span>;
    } else {
      const hours = Math.floor(waitingTime / 60);
      const minutes = waitingTime % 60;
      return <span className='NotReadyNow'>{hours}시간 {minutes}분 후 충전 가능</span>;
    }
  };

  return (
    <div className='SelectParking'>
      <div className='SelectParking_contents_box'>
        <div className='SelectParking_contents1'>주차장을</div>
        <div className='SelectParking_contents2'>선택해 주세요.</div>
      </div>

      <div className='Content_Area'> {/* 스크롤 가능한 콘텐츠 영역 */}
        <input
          type="text"
          placeholder="주차장 검색..."
          className='SearchInput'
          value={searchTerm}
          onChange={(e) => setSearchTerm(e.target.value)} // 검색어 상태 업데이트
        />
        <div className='Botton_box_select'>
          <button
            onClick={() => handleSort('distance')}
            className={`selectparkingbutton ${activeButton === 'distance' ? 'active' : ''}`}
          >
            거리순
          </button>
          <button
            onClick={() => handleSort('time')}
            className={`selectparkingbutton2 ${activeButton === 'time' ? 'active' : ''}`}
          >
            시간순
          </button>
        </div>

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
                  <div className='ParkingDistance'>
                    {lot.distance >= 1000 
                      ? `${(lot.distance / 1000).toFixed(1)}km`  
                      : `${Math.floor(lot.distance)}m`}          
                  </div>
                  <div>{formatWaitingTime(lot.waitingTime)}</div> {/* 대기 시간 포맷 */}
                </div>
              </li>
            ))
          ) : (
            <li className='NoResults'>일치하는 주차장이 없습니다</li>
          )}
        </ul>
        {error && <div className='Error'>{error}</div>} {/* 에러 메시지 표시 */}
      </div>
    </div>
  );
}

export default SelectParking;
