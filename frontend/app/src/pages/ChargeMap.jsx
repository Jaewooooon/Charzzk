import React, { useEffect, useState } from 'react';
import axios from 'axios';
import '../styles/ChargeMap.css';
import GoBackButton from '../components/GobackButton.jsx';
import MyLocation from '../assets/MyCarLocation.png'; // 이미지 임포트
import ParkingLocation from '../assets/marker.png'; // 주차장 마커 이미지 임포트
const { kakao } = window;

function Kakao() {
  const [map, setMap] = useState(null);
  const [innerWidth, setInnerWidth] = useState(window.innerWidth);
  const [innerHeight, setInnerHeight] = useState(window.innerHeight);
  const [parkingLots, setParkingLots] = useState([]);
  const [menuHeight, setMenuHeight] = useState(350); // 초기 메뉴 높이
  const [startY, setStartY] = useState(0); // 터치 시작 Y좌표
  const [startHeight, setStartHeight] = useState(200); // 터치 시작 시점의 높이
  const [isExpanded, setIsExpanded] = useState(false); // 메뉴 확장 상태
  const [location, setLocation] = useState({ latitude: null, longitude: null });
  const [error, setError] = useState(null);
  const [searchTerm, setSearchTerm] = useState(''); // 검색어 상태 추가

  useEffect(() => {
    const resizeListener = () => {
      setInnerWidth(window.innerWidth);
      setInnerHeight(window.innerHeight);
    };
    window.addEventListener("resize", resizeListener);

    if (navigator.geolocation) {
      navigator.geolocation.getCurrentPosition(
        (position) => {
          const { latitude, longitude } = position.coords;
          setLocation({ latitude, longitude });
          
          const container = document.getElementById('KakaoMap');
          const options = {
            center: new kakao.maps.LatLng(latitude, longitude), // 사용자의 현재 위치로 중심 설정
            level: 2,
          };

          const newMap = new kakao.maps.Map(container, options);
          setMap(newMap);

          // 현재 위치에 이미지 마커 추가
          const markerPosition = new kakao.maps.LatLng(latitude, longitude);
          const markerImage = new kakao.maps.MarkerImage(MyLocation, new kakao.maps.Size(100, 80));

          const marker = new kakao.maps.Marker({
            position: markerPosition,
            image: markerImage, // 커스텀 마커 이미지 설정
            map: newMap,
            title: "현재 위치",
          });

          // 주차장 데이터를 가져오는 API 호출
          axios.get('https://j11c208.p.ssafy.io/api/v1/parking-lot', {
            params: {
              latitude: latitude, // 현재 위치의 위도
              longitude: longitude // 현재 위치의 경도
            }
          })
          .then(response => {
            const lots = response.data.data; // data 필드에서 주차장 리스트 추출
            setParkingLots(lots); // 주차장 좌표 저장
          })
          .catch(error => {
            console.error("Error fetching parking lots:", error);
          });
        },
        (error) => {
          setError("Error retrieving location: " + error.message);
        }
      );
    } else {
      setError("Geolocation is not supported by this browser.");
    }

    return () => {
      window.removeEventListener('resize', resizeListener);
    };
  }, []);

  useEffect(() => {
    if (map && parkingLots.length > 0) {
      parkingLots.forEach(lot => {
        const markerPosition = new kakao.maps.LatLng(lot.location.latitude, lot.location.longitude);

        // 주차장 마커 이미지 설정
        const parkingMarkerImage = new kakao.maps.MarkerImage(ParkingLocation, new kakao.maps.Size(100, 50));

        const marker = new kakao.maps.Marker({
          position: markerPosition,
          image: parkingMarkerImage, // 주차장 마커 이미지 적용
          map: map,
        });
        marker.setMap(map); // 마커를 지도에 표시
      });
    }
  }, [map, parkingLots]);

  const handleTouchStart = (e) => {
    setStartY(e.touches[0].clientY); // 터치 시작 위치 기록
    setStartHeight(menuHeight); // 터치 시작 시점의 높이 저장
  };

  const handleTouchMove = (e) => {
    const currentY = e.touches[0].clientY;
    const distance = startY - currentY; // 터치 이동 거리
    const newHeight = startHeight + distance;

    // 손가락 이동에 따라 높이 설정 (최소 100px, 최대 전체 화면)
    if (newHeight >= 100 && newHeight <= innerHeight) {
      setMenuHeight(newHeight);
    }
  };

  const handleTouchEnd = (e) => {
    const currentY = e.changedTouches[0].clientY;
    const distance = startY - currentY;
    const duration = e.timeStamp - e.target.dataset.startTime;

    // 빠르게 스와이프하면 전체 화면으로 확장
    if (duration < 300 && distance > 50) {
      setIsExpanded(true);
      setMenuHeight(innerHeight);
    } else if (menuHeight > innerHeight / 2) {
      // 천천히 움직여도 일정 높이 이상이면 확장
      setIsExpanded(true);
      setMenuHeight(innerHeight);
    } else {
      // 그렇지 않으면 원래 높이로 복구
      setIsExpanded(false);
      setMenuHeight(350);
    }
  };

  // 필터링된 주차장 목록을 검색어에 따라 계산
  const filteredParkingLots = parkingLots.filter(lot =>
    lot.name.toLowerCase().includes(searchTerm.toLowerCase())
  );

  return (
    <div style={{ position: 'relative' }}>
      <GoBackButton />
      <input 
        type="text" 
        placeholder="Search location..." 
        className='ChargeMap_Search' 
        value={searchTerm} // 입력된 검색어와 상태를 동기화
        onChange={(e) => setSearchTerm(e.target.value)} // 검색어 상태 업데이트
      />
      <div id="KakaoMap" style={{ width: innerWidth, height: innerHeight }}></div>
      
      {/* 터치 이벤트가 적용된 coordinates-box */}
      <div
        className={`coordinates-box ${isExpanded ? 'expanded' : ''}`}
        style={{ height: `${menuHeight}px` }} // 높이를 동적으로 설정
        onTouchStart={handleTouchStart}
        onTouchMove={handleTouchMove}
        onTouchEnd={handleTouchEnd}
        data-start-time={Date.now()} // 스와이프 시작 시간 기록
      >
        <h3>Parking Lot Coordinates</h3>
        <ul>
          {filteredParkingLots.length > 0 ? (
            filteredParkingLots.map((lot, index) => (
              <li key={index}>
                <img src={lot.image} alt="주차장 이미지" style={{ width: '100px', height: '100px' }} />
                {lot.name}  {lot.distance}m
              </li>
            ))
          ) : (
            <li>일치하는 주차장이 없습니다</li>
          )}
        </ul>
      </div>
    </div>
  );
}

export default Kakao;
