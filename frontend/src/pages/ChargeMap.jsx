import React, { useEffect, useState } from 'react';
import MarkerImage from '../assets/marker.png'; // 사용자 정의 마커 이미지 가져오기

const { kakao } = window;

function Kakao() {
  const [map, setMap] = useState(null);
  const [innerWidth, setInnerWidth] = useState(window.innerWidth);
  const [innerHeight, setInnerHeight] = useState(window.innerHeight);
  const [searchQuery, setSearchQuery] = useState(''); // 검색어 상태 추가
  const [selectedMarkerIndex, setSelectedMarkerIndex] = useState(null); // 선택된 마커의 인덱스 상태 추가

  const positions = [
    {
      title: "삼성 사업장",
      latlng: new kakao.maps.LatLng(35.205580, 126.81131),
      address: "광주광역시 광산구 가로 111-1",
      distance: "120m"
    },
    {
      title: "새로운 마커 1",
      latlng: new kakao.maps.LatLng(35.207580, 126.81232),
      address: "광주광역시 광산구 가로 111-1",
      distance: "700m"
    },
    {
      title: "새로운 마커 2",
      latlng: new kakao.maps.LatLng(35.209880, 126.81333),
      address: "광주광역시 광산구 가로 111-1",
      distance: "1.2km"
    }
  ];

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

    // 사용자 정의 마커 이미지 설정
    const imageSrc = MarkerImage; // 마커 이미지 소스
    const imageSize = new kakao.maps.Size(40, 40); // 마커 이미지 크기 설정
    const imageOption = { offset: new kakao.maps.Point(12, 35) }; // 마커 이미지의 중심 좌표 설정

    const markerImage = new kakao.maps.MarkerImage(imageSrc, imageSize, imageOption);

    positions.forEach((position, index) => {
      const marker = new kakao.maps.Marker({
        map: newMap,
        position: position.latlng,
        title: position.title,
        image: markerImage // 사용자 정의 마커 이미지 적용
      });

      // 마커 클릭 이벤트 추가
      kakao.maps.event.addListener(marker, 'click', () => {
        setSelectedMarkerIndex(index); // 선택된 마커의 인덱스를 상태로 설정
        // 마커 이미지 확대
        const selectedMarkerImage = new kakao.maps.MarkerImage(
          imageSrc,
          new kakao.maps.Size(60, 50), // 선택된 마커 크기 변경
          imageOption
        );
        marker.setImage(selectedMarkerImage); // 마커 이미지 업데이트
      });
    });

    return () => {
      window.removeEventListener('resize', resizeListener);
    };
  }, []);

  // 검색어에 따라 필터링된 마커 목록 생성
  const filteredPositions = positions.filter(position =>
    position.title.toLowerCase().includes(searchQuery.toLowerCase())
  );

  return (
    <div style={{ position: 'relative' }}>
      {/* 검색 창 */}
      <input
        type="text"
        placeholder="검색"
        value={searchQuery}
        onChange={(e) => setSearchQuery(e.target.value)} // 검색어 상태 업데이트
        style={{
          position: 'absolute',
          top: '20px',
          left: '50%',
          transform: 'translateX(-50%)',
          zIndex: 1000,
          width: '300px',
          height: '40px',
          padding: '10px',
          borderRadius: '8px',
          boxShadow: '0 2px 8px rgba(0, 0, 0, 0.1)',
          border: 'none',
          outline: 'none'
        }}
      />

      {/* 지도 표시 */}
      <div id="KakaoMap" style={{ width: '100vw', height: '100vh' }}></div>

      {/* 마커 목록 */}
      <div style={{
        position: 'absolute',
        bottom: '0px',
        left: '50%',
        transform: 'translateX(-50%)',
        zIndex: 1000,
        width: '90%',
        maxHeight: '300px',
        backgroundColor: '#fff',
        borderRadius: '16px',
        boxShadow: '0 2px 8px rgba(0, 0, 0, 0.2)',
        overflowY: 'auto',
        padding: '20px'
      }}>
        <h3>가까운 충전소</h3>
        {filteredPositions.map((position, index) => ( // 필터링된 목록을 렌더링
          <div key={index} style={{
            display: 'flex',
            alignItems: 'center',
            marginBottom: '15px',
            backgroundColor: selectedMarkerIndex === index ? '#f0f0f0' : 'transparent', // 선택된 마커 배경색 변경
            borderRadius: '8px',
            padding: '10px'
          }}>
            <div style={{ width: '50px', height: '50px', backgroundColor: '#e0e0e0', borderRadius: '8px', marginRight: '10px' }}>
              <img src={MarkerImage} alt={position.title} style={{ width: '100%', height: '100%', objectFit: 'cover' }} />
            </div>
            <div>
              <h4 style={{ margin: 0 }}>{position.title}</h4>
              <p style={{ margin: 0, color: '#666' }}>{position.address}</p>
              <p style={{ margin: 0, color: '#999' }}>{position.distance}</p>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}

export default Kakao;