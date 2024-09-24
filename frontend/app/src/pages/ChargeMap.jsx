import React, { useEffect, useState } from 'react';
import axios from 'axios';
import '../styles/ChargeMap.css';

const { kakao } = window;

function Kakao() {
  const [map, setMap] = useState(null);
  const [innerWidth, setInnerWidth] = useState(window.innerWidth);
  const [innerHeight, setInnerHeight] = useState(window.innerHeight);
  const [parkingLots, setParkingLots] = useState([]);
  const [menuHeight, setMenuHeight] = useState(200); // 초기 메뉴 높이
  const [startY, setStartY] = useState(0); // 터치 시작 Y좌표
  const [startHeight, setStartHeight] = useState(200); // 터치 시작 시점의 높이
  const [isExpanded, setIsExpanded] = useState(false); // 메뉴 확장 상태

  useEffect(() => {
    const resizeListener = () => {
      setInnerWidth(window.innerWidth);
      setInnerHeight(window.innerHeight);
    };
    window.addEventListener("resize", resizeListener);

    const container = document.getElementById('KakaoMap');
    const options = {
      center: new kakao.maps.LatLng(35.205580, 126.811458),
      level: 2
    };

    const newMap = new kakao.maps.Map(container, options);
    setMap(newMap);

    // API 요청
    axios.get('https://j11c208.p.ssafy.io/api/v1/parking-lot')
      .then(response => {
        const lots = response.data;
        setParkingLots(lots); // 주차장 좌표 저장
      })
      .catch(error => {
        console.error("Error fetching parking lots:", error);
      });

    return () => {
      window.removeEventListener('resize', resizeListener);
    };
  }, []);

  useEffect(() => {
    if (map && parkingLots.length > 0) {
      parkingLots.forEach(lot => {
        const markerPosition = new kakao.maps.LatLng(lot.latitude, lot.longitude);
        const marker = new kakao.maps.Marker({
          position: markerPosition,
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
      setMenuHeight(200);
    }
  };

  return (
    <div style={{ position: 'relative' }}>
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
          {parkingLots.length > 0 ? (
            parkingLots.map((lot, index) => (
              <li key={index}>
                <strong>Latitude:</strong> {lot.latitude}, <strong>Longitude:</strong> {lot.longitude}
              </li>
            ))
          ) : (
            <li>데이터가 없습니다</li>
          )}
        </ul>
      </div>
    </div>
  );
}

export default Kakao;
