import React, { useEffect, useState } from 'react';
import axios from 'axios';
import FullCalendar from '@fullcalendar/react'; // FullCalendar component
import dayGridPlugin from '@fullcalendar/daygrid'; // Plugin for day grid view
import timeGridPlugin from '@fullcalendar/timegrid'; // Plugin for time grid (day) view
import '../styles/MapManagement.css';
import ParkingLocation from '../assets/marker.png';
import MyLocation from '../assets/ManagementLocation.png';

const { kakao } = window;

const MapManagement = () => {
  const [map, setMap] = useState(null);
  const [location, setLocation] = useState(null);
  const [parkingLots, setParkingLots] = useState([]);
  const [filteredParkingLots, setFilteredParkingLots] = useState([]);
  const [selectedLot, setSelectedLot] = useState(null);
  const [chargers, setChargers] = useState([]);
  const [selectedSerial, setSelectedSerial] = useState(""); 
  const [chargerStatus, setChargerStatus] = useState(null);
  const [reservations, setReservations] = useState([]); // 예약 정보를 저장할 state
  const [error, setError] = useState(null);

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
    if (navigator.geolocation) {
      navigator.geolocation.getCurrentPosition(
        (position) => {
          const { latitude, longitude } = position.coords;
          setLocation({ latitude, longitude });

          const container = document.getElementById('KakaoMap');
          const options = {
            center: new kakao.maps.LatLng(latitude, longitude),
            level: 2,
          };

          const newMap = new kakao.maps.Map(container, options);
          setMap(newMap);

          const markerPosition = new kakao.maps.LatLng(latitude, longitude);
          const markerImage = new kakao.maps.MarkerImage(MyLocation, new kakao.maps.Size(100, 80));

          const marker = new kakao.maps.Marker({
            position: markerPosition,
            image: markerImage,
            map: newMap,
            title: "현재 위치",
          });

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
              setFilteredParkingLots(resolvedLots);
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

        marker.setMap(map); 

        kakao.maps.event.addListener(marker, 'click', () => {
          handleRobotCheck(lot.id, index);
        });
      });
    }
  }, [map, parkingLots]);

  const fetchChargerData = (parkingLotId) => {
    axios.get(`https://j11c208.p.ssafy.io/api/v1/parking-lot/${parkingLotId}/chargers`)
      .then(response => {
        setChargers(response.data.data);
      })
      .catch(error => {
        console.error("Error fetching chargers:", error);
      });
  };

  const handleRobotCheck = (parkingLotId, index) => {
    // 이미 선택된 주차장이 클릭되었으면 선택 해제
    if (selectedLot === index) {
      setSelectedLot(null); // 선택 해제
      setChargers([]); // 충전기 정보 초기화
      setChargerStatus(null); // 충전기 상태 초기화
      setReservations([]); // 예약 정보 초기화
    } else {
      setSelectedLot(index);
      fetchChargerData(parkingLotId);
    }
  };

  const handleSerialChange = (e) => {
    const serialNumber = e.target.value;
    setSelectedSerial(serialNumber);
    
    const selectedCharger = chargers.find(charger => charger.serialNumber === serialNumber);
    if (selectedCharger) {
      console.log("Charger Status:", selectedCharger);
      setChargerStatus(selectedCharger);
      setReservations(selectedCharger.reservations); // 예약 정보를 state에 저장
    }
  };

  const translateChargerStatus = (status) => {
    switch (status) {
      case 'CAR_CHARGING':
        return '충전 중';
      case 'WAITING':
        return '대기 중';
      case 'ERROR':
        return '수리 중';
      default:
        return '알 수 없음';
    }
  };

  return (
    <div>
      <div id="KakaoMap" style={{ width: '100%', height: '90vh' }}></div>
      <ul className='ParkingListBox'>
        <div className='ParkingListTitle'>주차장 목록</div>
        {filteredParkingLots.length > 0 ? (
          filteredParkingLots.map((lot, index) => (
            <li key={index} className={`parking_box ${selectedLot === index ? 'active' : ''}`} onClick={() => handleRobotCheck(lot.id, index)}>
              <img src={lot.image} alt="주차장 이미지" style={{ width: '90px', height: '77px' }} />
              <div className='parking_contents'>
                <h3 className='parking_name'>{lot.name}</h3>
                <div className='parking_address'>{lot.address}</div>
              </div>

              {selectedLot === index && chargers.length > 0 && (
                <div className='robot_info'>
                  <h4 className='MapChargeRobotacontent'>Robot Serial Number</h4>
                  <select value={selectedSerial} onChange={handleSerialChange} className='Select_Serial_Number'>
                    <option value="" className='Serial_option'>시리얼 넘버 선택</option>
                    {chargers.map((charger) => (
                      <option key={charger.chargerId} value={charger.serialNumber} className='Serial_option'>
                        {charger.serialNumber}
                      </option>
                    ))}
                  </select>

                  {chargerStatus && (
                    <div className='charger_status'>
                      <p className='RobotState'>로봇 상태: {translateChargerStatus(chargerStatus.status)}</p>
                    </div>
                  )}

                  {chargerStatus && reservations.length > 0 && (
                    <div>
                      <div style={{ width: '60%', height: '400px' }}>
                        <FullCalendar
                          plugins={[dayGridPlugin, timeGridPlugin]} // timeGridPlugin 추가
                          initialView="timeGridDay" // 하루 일정 뷰로 변경
                          events={reservations.map((reservation) => ({
                            title: `차량 번호: ${reservation.carNumber}`,
                            start: reservation.startTime,
                            end: reservation.endTime,
                          }))} 
                        />
                      </div>
                    </div>
                  )}
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
};

export default MapManagement;
