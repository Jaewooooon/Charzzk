import React, { useEffect, useState } from 'react';
import axios from 'axios';
import '../styles/MapManagement.css';
import ParkingLocation from '../assets/marker.png';
import MyLocation from '../assets/ManagementLocation.png';

const { kakao } = window;

const MapManagement = () => {
  const [map, setMap] = useState(null);
  const [location, setLocation] = useState(null);
  const [parkingLots, setParkingLots] = useState([]);
  const [filteredParkingLots, setFilteredParkingLots] = useState([]);
  const [error, setError] = useState(null);
  const [innerWidth, setInnerWidth] = useState(window.innerWidth);
  const [innerHeight, setInnerHeight] = useState(window.innerHeight);
  const [selectedLot, setSelectedLot] = useState(null);
  
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
    window.addEventListener('resize', resizeListener);

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
              latitude: latitude,
              longitude: longitude
            }
          })
          .then(response => {
            const lots = response.data.data;

            const updatedLots = lots.map(lot => {
              return new Promise((resolve) => {
                getAddressFromCoords(lot.location.latitude, lot.location.longitude, (address) => {
                  resolve({ ...lot, address });
                });
              });
            });

            Promise.all(updatedLots).then((resolvedLots) => {
              setParkingLots(resolvedLots);
              setFilteredParkingLots(resolvedLots);  // 필터링된 주차장 리스트 업데이트
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
      parkingLots.forEach((lot, index) => {
        const markerPosition = new kakao.maps.LatLng(lot.location.latitude, lot.location.longitude);
        const parkingMarkerImage = new kakao.maps.MarkerImage(ParkingLocation, new kakao.maps.Size(100, 50));

        const marker = new kakao.maps.Marker({
          position: markerPosition,
          image: parkingMarkerImage,
          map: map,
        });

        marker.setMap(map); // 마커를 지도에 표시

        // 마커 클릭 이벤트 추가
        kakao.maps.event.addListener(marker, 'click', () => {
          handleRobotCheck(index); // 마커 클릭 시 주차장 정보 표시
        });
      });
    }
  }, [map, parkingLots]);

  // 버튼 클릭 또는 마커 클릭 시 주차장 정보 표시
  const handleRobotCheck = (index) => {
    setSelectedLot((prevSelectedLot) => (prevSelectedLot === index ? null : index));
  };

  return (
    <div>
      <div id="KakaoMap" style={{ width: '100%', height: '90vh' }}></div>
      <ul className='ParkingListBox'>
      <div className='ParkingListTitle'>주차장 목록</div>
        {filteredParkingLots.length > 0 ? (
          filteredParkingLots.map((lot, index) => (
            <li key={index} className={`parking_box ${selectedLot === index ? 'active' : ''}`} onClick={() => handleRobotCheck(index)}>
              <img src={lot.image} alt="주차장 이미지" style={{ width: '90px', height: '77px' }} />
              <div className='parking_contents'>
                <h3 className='parking_name'>{lot.name}</h3>
                <div className='parking_address'>{lot.address}</div>

              </div>

              {selectedLot === index && (
                  <div className='robot_info'>
                  <h4>Robot Information</h4>
                  <p>주차장: {lot.name}</p>
                  <p>상태: 충전중 </p>
                </div>
              )}

            </li>
          ))
        ) : (
          <li>일치하는 주차장이 없습니다</li>
        )}
      </ul>
    </div>
  );
}

export default MapManagement;
