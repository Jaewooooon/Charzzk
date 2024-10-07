import React, { useEffect, useState } from 'react';
import { useNavigate } from 'react-router-dom';
import '../../styles/CarManagement.css';
import CarSample from '../../assets/car_sample.png';
import Patch from '../../assets/Patch_Logo.png';
import Add from '../../assets/Add_Logo.png';
import Delete from '../../assets/Delete_Logo.png';
import DeleteModal from '../../components/DeleteModal.jsx';
import PatchModal from '../../components/PatchModal.jsx';
import AddCarModal from '../../components/AddModal.jsx'; // 새로운 모달 컴포넌트 추가
import { CircularProgressbar, buildStyles } from 'react-circular-progressbar';
import 'react-circular-progressbar/dist/styles.css';
import { useSpring, animated } from '@react-spring/web';
import GoBackButton from '../../components/GobackButton';
import axios from 'axios';
import { accessTokenState } from '../../recoil/LoginAtom.jsx';
import { currentIndexState } from '../../recoil/CurrentIndex.jsx';
import { useRecoilValue, useRecoilState } from 'recoil';

const AnimatedCircularProgressbar = animated(CircularProgressbar);

function CarManagement() {
  const [carData, setCarData] = useState([]); // 초기값을 빈 배열로 설정
  const [carTypes, setCarTypes] = useState([]); // 차량 타입 목록 상태 추가
  const accessToken = useRecoilValue(accessTokenState); 
  const progressValue = 60;
  const [currentIndex, setCurrentIndex] = useRecoilState(currentIndexState);
  const [modalIsOpen, setIsOpen] = useState(false); 
  const [patchModalIsOpen, setPatchModalIsOpen] = useState(false);
  const [addCarModalIsOpen, setAddCarModalIsOpen] = useState(false); // 차량 추가 모달 상태
  const navigate = useNavigate();
  const props = useSpring({
    from: { value: 0 },
    to: { value: progressValue },
    config: { duration: 500 },
  });

  const fetchCarData = async () => {
    try {
      const response = await axios.get('https://j11c208.p.ssafy.io/api/v1/cars/me', {
        headers: {
          Authorization: `Bearer ${accessToken}`,
        },
      });
      
      if (response.data.code === 200) {
        setCarData(response.data.data); 
      } else {
        console.error('Error fetching car data:', response.data.message);
      }
    } catch (error) {
      console.error('Error fetching data:', error);
    }
  };

  const fetchCarTypes = async () => {
    try {
      const response = await axios.get('https://j11c208.p.ssafy.io/api/v1/car-types', {
        headers: {
          Authorization: `Bearer ${accessToken}`,
        },
      });
      
      if (response.data.code === 200) {
        setCarTypes(response.data.data); // 차량 타입 데이터 저장
      } else {
        console.error('Error fetching car types:', response.data.message);
      }
    } catch (error) {
      console.error('Error fetching car types:', error);
    }
  };

  useEffect(() => {
    fetchCarData();
    fetchCarTypes(); // 컴포넌트 로드 시 차량 타입 데이터 가져오기
  }, []);

  // 모달 열기 함수
  const openModal = () => setIsOpen(true);
  const closeModal = () => setIsOpen(false);

  // 패치 모달 열기 함수
  const openPatchModal = () => setPatchModalIsOpen(true);
  const closePatchModal = () => setPatchModalIsOpen(false);

  // 차량 추가 모달 열기 함수
  const openAddCarModal = () => setAddCarModalIsOpen(true);
  const closeAddCarModal = () => setAddCarModalIsOpen(false);

  // 차량 삭제 함수
  const deleteCar = async () => {
    try {
      const response = await axios.delete(`https://j11c208.p.ssafy.io/api/v1/cars/${carData[currentIndex].id}`, {
        headers: {
          Authorization: `Bearer ${accessToken}`,
        },
      });
      if (response.data.code === 200) {
        alert('차량이 성공적으로 삭제되었습니다.');
        await fetchCarData(); // 삭제 후 차량 데이터를 다시 불러옴
        
        // 삭제 후 현재 인덱스를 업데이트
        if (carData.length === 1) {
          setCurrentIndex(0); // 마지막 차량이 삭제된 경우 인덱스를 0으로 설정
        } else {
          setCurrentIndex((prevIndex) => (prevIndex >= carData.length ? carData.length - 1 : prevIndex));
        }
  
        closeModal();   // 모달 닫기
        navigate('/'); 
      } else {
        console.error('Error deleting car:', response.data.message);
      }
    } catch (error) {
      console.error('Error deleting car:', error);
    }
  };

  // 차량 정보 수정 PATCH 요청 함수
  const patchCar = async (updatedCarData) => {
    try {
      const response = await axios.patch(
        `https://j11c208.p.ssafy.io/api/v1/cars/${carData[currentIndex].id}`,
        updatedCarData,
        {
          headers: {
            Authorization: `Bearer ${accessToken}`,
          },
        }
      );
      if (response.data.code === 200) {
        alert('차량 정보가 성공적으로 수정되었습니다.');
        fetchCarData(); // 수정 후 차량 데이터를 다시 불러옴
        closePatchModal(); // 모달 닫기
      } else {
        console.error('Error updating car:', response.data.message);
      }
    } catch (error) {
      console.error('Error updating car:', error);
    }
  };

  // 차량 추가 POST 요청 함수
  const addCar = async (newCarData) => {
    try {
      const response = await axios.post(
        'https://j11c208.p.ssafy.io/api/v1/cars',
        newCarData,
        {
          headers: {
            Authorization: `Bearer ${accessToken}`,
          },
        }
      );
      if (response.data.code === 200) {
        console.log(newCarData);
        alert('차량이 성공적으로 추가되었습니다.');
        fetchCarData(); // 추가 후 차량 데이터를 다시 불러옴
        closeAddCarModal(); // 모달 닫기
      } else {
        console.error('Error adding car:', response.data.message);
      }
    } catch (error) {
      console.error('Error adding car:', error);
    }
  };

  return (
    <div className='CarManagement_ContainerBox'>
      <GoBackButton />
      <div className='CarManagement_ButtonBox'>
        <button className='DeleteCar_button' onClick={openModal}><img src={Delete} alt="삭제" /></button>
        <button className='PatchCar_button' onClick={openPatchModal}><img src={Patch} alt="변경" /></button>
        <button className='AddCar_button' onClick={openAddCarModal}><img src={Add} alt="추가" /></button>
      </div>

      {carData.length > 0 ? (
        <>
          <div className='Car_ImageBox'>
            <img src={carData[currentIndex].carType.image} alt="Car Image" className="Car_Image_Management" />
          </div>

          <div className='Circular_ProgressBox'>
            <AnimatedCircularProgressbar
              value={props.value} 
              text={props.value.to(val => `${Math.round(val)}%`)} 
              className="circular-progressbar"
              styles={buildStyles({
                textColor: 'rgba(214, 214, 214, 0.7)',
                pathColor: '#65F9D1',
                trailColor: 'rgba(214, 214, 214, 0.5)',
                textSize: '15px',
                strokeLinecap: 'butt', 
              })}
              strokeWidth={15} 
            />
          </div>

          <div className='Car_Info'>
            <h2>닉네임: {carData[currentIndex].nickname}</h2>
            <h3>차량 이름: {carData[currentIndex].carType.name}</h3>
          </div>
        </>
      ) : (
        <p>차량 정보를 로드 중입니다...</p>
      )}

      {/* 모달들 */}
      <DeleteModal 
        isOpen={modalIsOpen} 
        onRequestClose={closeModal} 
        onConfirm={deleteCar} 
      />

      <PatchModal 
        isOpen={patchModalIsOpen} 
        onRequestClose={closePatchModal} 
        onConfirm={patchCar} 
        carData={carData[currentIndex]} 
      />

      <AddCarModal 
        isOpen={addCarModalIsOpen}
        onRequestClose={closeAddCarModal}
        onConfirm={addCar} 
        carTypes={carTypes} // 차량 타입 리스트 전달
      />
    </div>
  );
}

export default CarManagement;
