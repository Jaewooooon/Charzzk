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
import GoBackButton from '../../components/GobackButton';
import axios from 'axios';
import { accessTokenState } from '../../recoil/LoginAtom.jsx';
import { currentIndexState } from '../../recoil/CurrentIndex.jsx';
import { useRecoilValue, useRecoilState } from 'recoil';

function CarManagement() {
  const [carData, setCarData] = useState([]);
  const [carTypes, setCarTypes] = useState([]);
  const accessToken = useRecoilValue(accessTokenState);
  const [currentIndex, setCurrentIndex] = useRecoilState(currentIndexState);
  const [modalIsOpen, setIsOpen] = useState(false);
  const [patchModalIsOpen, setPatchModalIsOpen] = useState(false);
  const [addCarModalIsOpen, setAddCarModalIsOpen] = useState(false);
  const navigate = useNavigate();

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
        setCarTypes(response.data.data);
      } else {
        console.error('Error fetching car types:', response.data.message);
      }
    } catch (error) {
      console.error('Error fetching car types:', error);
    }
  };

  useEffect(() => {
    fetchCarData();
    fetchCarTypes();
  }, []);

  const openModal = () => setIsOpen(true);
  const closeModal = () => setIsOpen(false);
  const openPatchModal = () => setPatchModalIsOpen(true);
  const closePatchModal = () => setPatchModalIsOpen(false);
  const openAddCarModal = () => setAddCarModalIsOpen(true);
  const closeAddCarModal = () => setAddCarModalIsOpen(false);

  const deleteCar = async () => {
    try {
      const response = await axios.delete(`https://j11c208.p.ssafy.io/api/v1/cars/${carData[currentIndex].id}`, {
        headers: {
          Authorization: `Bearer ${accessToken}`,
        },
      });
      if (response.data.code === 200) {
        alert('차량이 성공적으로 삭제되었습니다.');
        await fetchCarData();

        if (carData.length === 1) {
          setCurrentIndex(0);
        } else {
          setCurrentIndex((prevIndex) => (prevIndex >= carData.length ? carData.length - 1 : prevIndex));
        }

        closeModal();
        navigate('/');
      } else {
        console.error('Error deleting car:', response.data.message);
      }
    } catch (error) {
      console.error('Error deleting car:', error);
    }
  };

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
        fetchCarData();
        closePatchModal();
      } else {
        console.error('Error updating car:', response.data.message);
      }
    } catch (error) {
      console.error('Error updating car:', error);
    }
  };

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
        alert('차량이 성공적으로 추가되었습니다.');
        fetchCarData();
        closeAddCarModal();
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
            <h2 className='Car_Nickname'>{carData[currentIndex].nickname}</h2>
            <div className='MonthPayment_Contents'>
              <div className='Battery_Management'>
                {[...Array(10)].map((_, index) => {
                  const batteryLevel = carData[currentIndex]?.battery || 0;
                  let batteryClass = 'CarNoChargeBattery_img';

                  if (batteryLevel === 100) {
                    batteryClass = 'CarChargeBattery_img1';
                  } else if (batteryLevel >= 70) {
                    batteryClass = 'CarChargeBattery_img2';
                  } else if (batteryLevel >= 30) {
                    batteryClass = 'CarChargeBattery_img3';
                  } else if (batteryLevel > 0) {
                    batteryClass = 'CarChargeBattery_img4';
                  }

                  return (
                    <div 
                      key={index} 
                      className={batteryLevel >= (10 - index) * 10 
                        ? batteryClass 
                        : 'CarNoChargeBattery_img'}
                    ></div>
                  );
                })}
              </div>
              <div className='Car_BattreryBox'>              
                <div className='Car_Battery'> {carData[currentIndex]?.battery || 0}</div>
              <div className='small_font2'>%</div></div>

            </div>
            <img src={carData[currentIndex].carType.image} alt="Car Image" className="Car_Image_Management" />
          </div>

          <div className='Car_Info'>
            <div className='CarManagement_Info2'>
              <div className='CarManagement_box1'>
                <div className='CarManagement_title'>차량 번호</div>
                <div className='CarManagement_content'>
                  <div className='small_font'>{carData[currentIndex]?.number?.slice(0, 3) || "N/A"}</div>
                  <div>{carData[currentIndex]?.number?.slice(3) || "N/A"}</div>
                </div>
              </div>

              <div className='CarManagement_box1'>
                <div className='CarManagement_title'>차량 종류</div>
                <div className='CarManagement_content2'>{carData[currentIndex].carType.name}</div>            
              </div>
            </div>
          </div>

          <div className='CarManagement_Info3'>
            <div className='CarManagement_box1'>
            <div className='CarManagement_title2'>이번달 충전요금</div>
            <div className='CarManagement_content'>{Math.floor(carData[currentIndex].chargeCost)}원
            </div>            
            </div>

            <div className='CarManagement_box1'>
            <div className='CarManagement_title2'>이번달 충전량</div>
            <div className='CarManagement_content'>{Math.floor(carData[currentIndex].chargeAmount)}KWH

            </div>            
            </div>
            </div>

        </>
      ) : (
        <p>차량 데이터가 없습니다.</p>
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
