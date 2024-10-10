import React, { useEffect, useState } from 'react';
import axios from 'axios';
import '../styles/ChargeMap.css';
import GoBackButton from '../components/GobackButton.jsx';
import MyLocation from '../assets/MyCarLocation.png'; // 이미지 임포트
import ParkingLocation from '../assets/marker.png'; // 주차장 마커 이미지 임포트
import Check from '../assets/Check.png';
import NoCheck from '../assets/NoCheck.png';

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
  const [sortedByDistance, setSortedByDistance] = useState(false); // 거리 정렬 상태
  const [isSorted, setIsSorted] = useState(false); // 버튼 클릭 여부 상태

  // 역지오코딩을 통해 주소 가져오기
  const getAddressFromCoords = (latitude, longitude, callback) => {
    const geocoder = new kakao.maps.services.Geocoder();

    const coord = new kakao.maps.LatLng(latitude, longitude);
    const callbackFn = (result, status) => {
      if (status === kakao.maps.services.Status.OK) {
        callback(result[0].address.address_name);
      } else {
        callback('주소를 가져올 수 없음');
      }
    };

    geocoder.coord2Address(coord.getLng(), coord.getLat(), callbackFn);
  };

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

            // 주차장 데이터에 주소를 추가하기 위해 각각의 좌표로 역지오코딩 요청
            const updatedLots = lots.map(lot => {
              return new Promise((resolve) => {
                getAddressFromCoords(lot.location.latitude, lot.location.longitude, (address) => {
                  resolve({ ...lot, address }); // 주소를 주차장 데이터에 추가
                });
              });
            });

            // 모든 주차장의 주소 데이터를 가져온 후 상태 업데이트
            Promise.all(updatedLots).then((resolvedLots) => {
              setParkingLots(resolvedLots);
            });

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

  // 주차장을 거리 순으로 정렬하는 함수
  const sortByDistance = () => {
    const sortedLots = [...parkingLots].sort((a, b) => Math.floor(a.distance) - Math.floor(b.distance));
    setParkingLots(sortedLots);
    setSortedByDistance(true); // 정렬 상태를 업데이트
    setIsSorted(!isSorted); // 이미지 토글 상태 변경
  };

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

    if (menuHeight > innerHeight *0.6) {
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
      >
        <div className='List_Title'>
          <h2 className='parking_title'>주변 주차장 조회 </h2>
        </div>
        
        <ul>
          {filteredParkingLots.length > 0 ? (
            filteredParkingLots.map((lot, index) => (
              <li key={index} className='parking_box'>
                <img src={lot.image} alt="주차장 이미지" style={{ width: '100px', height: '100px' }} />
                <div className='parking_contents'>
                  <h3 className='parking_name'>{lot.name}</h3>
                  <div>{lot.address}</div> {/* 주소 표시 */}
                  <div className='parking_distance'>{Math.floor(lot.distance)}m</div> {/* 소수점 버림 */}
                </div>
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
