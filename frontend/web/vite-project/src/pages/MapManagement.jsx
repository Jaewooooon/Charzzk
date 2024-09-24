import React, { useEffect, useState } from 'react';
import '../styles/MapManagement.css';

const { kakao } = window;

const MapManagement = () => {
  const [map, setMap] = useState(null);

  useEffect(() => {
    // 지도를 표시할 div의 id
    const container = document.getElementById('map'); 
    const options = {
      center: new kakao.maps.LatLng(37.5665, 126.9780), // 지도의 초기 중심 좌표 (서울 좌표)
      level: 3 // 지도의 초기 줌 레벨
    };

    // 지도 생성
    const map = new kakao.maps.Map(container, options);
    setMap(map); // map 상태에 지도 객체 저장

    // Clean-up function (컴포넌트 언마운트 시)
    return () => {
      // 필요에 따라 clean-up 작업 수행
      setMap(null);
    };
  }, []);

  return (

      <div id="map" style={{ width: '80%', height: '700px' }}></div>

  );
}

export default MapManagement;
