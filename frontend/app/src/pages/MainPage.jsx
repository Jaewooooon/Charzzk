import React, { useEffect, useState } from 'react';
import { Link } from 'react-router-dom';  
import '../styles/MainPage.css';
import CarSample from '../assets/car_sample.png';
import MypageCar from '../assets/MypageCar.png';
import MyPagePayment from '../assets/MypagePayment.png';
import MyPageReport from '../assets/MyPageReport.png';
import ChargeStatus from '../assets/ChargeStatus.png';
import ChargeStart from '../assets/ChargeStart.png';
import axios from 'axios'; 
import { useRecoilValue, useRecoilState } from 'recoil';
import { accessTokenState } from '../recoil/LoginAtom';
import { currentIndexState } from '../recoil/CurrentIndex';
import { useSwipeable } from 'react-swipeable';

const MainPage = () => {
  const [payment, setPayment] = useState(0);
  const [chargeamount, setChargeAmount] = useState(0);
  const [carData, setCarData] = useState([]);
  const [currentIndex, setCurrentIndex] = useRecoilState(currentIndexState); 
  const accessToken = useRecoilValue(accessTokenState); 

  const [touchStartX, setTouchStartX] = useState(0);
  const [touchEndX, setTouchEndX] = useState(0);

  const fetchCarData = async () => {
    try {
      const response = await axios.get('https://j11c208.p.ssafy.io/api/v1/cars/me', {
        headers: {
          Authorization: `Bearer ${accessToken}`,
        },
      });

      if (response.data.code === 200) {
        setCarData(response.data.data); 
        if (response.data.data.length > 0) {
          setPayment(response.data.data[currentIndex].chargeCost || 0);
          setChargeAmount(response.data.data[currentIndex].chargeAmount || 0);
        }
      } else {
        console.error('Error fetching car data:', response.data.message);
      }
    } catch (error) {
      console.error('Error fetching data:', error);
    }
  };

  useEffect(() => {
    fetchCarData();
  }, []);

  const handleTouchStart = (e) => {
    setTouchStartX(e.touches[0].clientX);
  };

  const handleTouchMove = (e) => {
    setTouchEndX(e.touches[0].clientX);
  };

  const handleTouchEnd = () => {
    if (touchStartX > touchEndX + 50) {
      // Swipe left
      nextImage();
    } else if (touchStartX < touchEndX - 50) {
      // Swipe right
      prevImage();
    }
  };

  const nextImage = () => {
    setCurrentIndex((prevIndex) => (prevIndex + 1) % carData.length);
  };

  const prevImage = () => {
    setCurrentIndex((prevIndex) => (prevIndex - 1 + carData.length) % carData.length);
  };

  useEffect(() => {
    if (carData.length > 0) {
      const currentCar = carData[currentIndex];
      if (currentCar) {
        setPayment(Math.floor(currentCar.chargeCost) || 0);
        setChargeAmount(Math.floor(currentCar.chargeAmount) || 0);
      }
    }
  }, [currentIndex, carData]);

  return (
    <div className='MainPage_ContainerBox'>
      {/* 동그라미 표시 */}
      <div className="dot-container">
        {carData.map((_, index) => (
          <div 
            key={index} 
            className={`dot ${currentIndex === index ? 'active' : ''}`} 
          />
        ))}
      </div>

      <div className='Car_Information' 
           onTouchStart={handleTouchStart} 
           onTouchMove={handleTouchMove} 
           onTouchEnd={handleTouchEnd}>
        {/* 차량 이미지 슬라이드 */}
        <img 
          src={carData.length > 0 ? carData[currentIndex]?.carType.image : CarSample} 
          alt={carData.length > 0 ? carData[currentIndex]?.carType.name : "Car Sample"} 
          className="Car_Image" 
        />
        
        <div className='Car_MonthInformation'>
          {/* 나머지 내용은 동일하게 유지 */}
          <div>
            <div className='Car_MonthInformation1'>배터리</div>
            <div className='MonthPayment_Contents'>
              <div>
                {[...Array(10)].map((_, index) => {
                  const batteryLevel = carData[currentIndex]?.battery || 0;
                  let batteryClass = 'NoChargeBattery_img'; 

                  if (batteryLevel === 100) {
                    batteryClass = 'ChargeBattery_img1'; 
                  } else if (batteryLevel >= 70) {
                    batteryClass = 'ChargeBattery_img2'; 
                  } else if (batteryLevel >= 30) {
                    batteryClass = 'ChargeBattery_img3'; 
                  } else if (batteryLevel > 0) {
                    batteryClass = 'ChargeBattery_img4'; 
                  }

                  return (
                    <div 
                      key={index} 
                      className={batteryLevel >= (10 - index) * 10 
                        ? batteryClass 
                        : 'NoChargeBattery_img'}
                    ></div>
                  );
                })}
              </div>
              <div>{carData[currentIndex]?.battery || 0}</div>
              <div className='small_font'>%</div>
            </div>
          </div>

          <div>
            <div className='Car_MonthInformation1'>차량 번호</div>
            <div className='MonthPayment_Contents'>
              <div className='small_font'>{carData[currentIndex]?.number?.slice(0, 3) || "N/A"}</div>
              <div>{carData[currentIndex]?.number?.slice(3) || "N/A"}</div>
            </div>
          </div>

          <div>
            <div className='Car_MonthInformation2'>이번달 충전량</div>
            <div className='MonthCharge_Contents'>
              <div className='chargeamount'>{chargeamount || 0}</div>
              <div className='small_font'>kWh</div>
            </div>
          </div>
        </div>
      </div>

      <div className='MainPage_ButtonBox'>
        <div className='ChargePage_Content'>충전페이지</div>
        <div className='ChargePage'>
          <div className="ButtonContainer">
            <Link to="/charge-map">
              <button className='ChargeFind_Button'>
                <img src={ChargeStatus} alt="ChargeStatus" className="ChargeStatus_img" />충전구역 조회
              </button>
            </Link>
            <Link to="/charge-start">
              <button className='ChargeStart_Button'>
                <div className='ChargeStart_Button_Content'>충전하기</div>
                <img src={ChargeStart} alt="ChargeStart" className="ChargeStart_Image" />
              </button>
            </Link>
          </div>

          <div>
            <Link to="/charge-status">
              <button className='ChargeStatus_Button'>충전현황 확인</button>
            </Link>
          </div>
        </div>
        
        <div className='MyPage_Content'>마이페이지</div>
        <div className='MyPage'>
          <Link to="/mypage/car-management">
            <button className='Mypage_Car'>
              <img src={MypageCar} alt="MypageCar" className="MypageCar_Image" />차량 관리
            </button>
          </Link>
          
          <Link to="/mypage/user-management">
            <button className='Mypage_Payment'>
              <img src={MyPagePayment} alt="MyPagePayment" className="MyPagePayment_Image" />내 사용내역
            </button>
          </Link>
          
          <Link to="/mypage/report-issue">
            <button className='Mypage_ReportIssue'>
              <img src={MyPageReport} alt="MyPageReport" className="MyPageReport_Image" />신고하기
            </button>
          </Link>
        </div>
      </div>

      <style>
        {`
          .dot-container {
            display: flex;
            justify-content: center;
            padding-top: 20px
          }
          .dot {
            height: 5px;
            width: 5px;
            margin: 0 5px;
            border-radius: 50%;
            background-color: lightgray;
            transition: background-color 0.3s;
          }
          .dot.active {
            background-color: #65F9D1;
          }
        `}
      </style>
    </div>
  );
};

export default MainPage;
