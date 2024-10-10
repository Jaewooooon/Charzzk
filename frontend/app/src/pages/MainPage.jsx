import React, { useEffect, useState } from 'react';
import { Link } from 'react-router-dom';  
import '../styles/MainPage.css';
import CarSample from '../assets/car_sample.png'; // 기본 이미지
import MypageCar from '../assets/MypageCar.png';
import MyPagePayment from '../assets/MypagePayment.png';
import MyPageReport from '../assets/MypageReport.png';
import ChargeStatus from '../assets/ChargeStatus.png';
import ChargeStart from '../assets/ChargeStart.png';
import Question from '../assets/Question.png';
import UserImg from '../assets/UserImg.png';
import axios from 'axios'; 
import { useRecoilValue, useRecoilState } from 'recoil';
import { accessTokenState } from '../recoil/LoginAtom';
import { currentIndexState } from '../recoil/CurrentIndex';
import { useSwipeable } from 'react-swipeable'; // react-swipeable 임포트

const MainPage = () => {
  const [payment, setPayment] = useState(0);
  const [chargeamount, setChargeAmount] = useState(0);
  const [carData, setCarData] = useState([]); // 차량 데이터를 배열로 저장
  const [currentIndex, setCurrentIndex] = useRecoilState(currentIndexState); // Recoil을 사용하여 currentIndex 상태 관리
  const accessToken = useRecoilValue(accessTokenState); 

  // 차량 데이터 가져오는 함수
  const fetchCarData = async () => {
    try {
      const response = await axios.get('https://j11c208.p.ssafy.io/api/v1/cars/me', {
        headers: {
          Authorization: `Bearer ${accessToken}`,
        },
      });

      if (response.data.code === 200) {
        setCarData(response.data.data); // 차량 데이터를 상태에 저장
        if (response.data.data.length > 0) {
          setPayment(response.data.data[currentIndex].chargeCost || 0); // 현재 차량의 충전 요금을 설정
          setChargeAmount(response.data.data[currentIndex].chargeAmount || 0); // 현재 차량의 충전량을 설정
        }
      } else {
        console.error('Error fetching car data:', response.data.message);
      }
    } catch (error) {
      console.error('Error fetching data:', error);
    }
  };

  // 컴포넌트 마운트 시 차량 데이터 가져오기
  useEffect(() => {
    fetchCarData();
  }, []);

  // 다음 이미지로 이동하는 함수
  const nextImage = () => {
    setCurrentIndex((prevIndex) => (prevIndex + 1) % carData.length);
  };

  // 이전 이미지로 이동하는 함수
  const prevImage = () => {
    setCurrentIndex((prevIndex) => (prevIndex - 1 + carData.length) % carData.length);
  };

  // Swipeable 설정
  const handlers = useSwipeable({
    onSwipedLeft: nextImage, // 왼쪽으로 스와이프 시 다음 이미지
    onSwipedRight: prevImage, // 오른쪽으로 스와이프 시 이전 이미지
    preventDefaultTouchmoveEvent: true,
    trackMouse: true,
  });

  // 차량 정보 업데이트
  useEffect(() => {
    if (carData.length > 0) {
      // currentIndex가 carData의 길이보다 크면 0으로 설정
      if (currentIndex >= carData.length) {
        setCurrentIndex(0);
      } else {
        const currentCar = carData[currentIndex]; // 현재 차량 데이터 가져오기
        if (currentCar) { // currentCar가 undefined가 아닐 경우만
          setPayment(Math.floor(currentCar.chargeCost) || 0);
          setChargeAmount(Math.floor(currentCar.chargeAmount) || 0);
        }
      }
    }
  }, [currentIndex, carData]);

  
  return (
    <>
    <div className='MainPage_ContainerBox'>
      <button className='Question_button'><img src={Question} alt="도움말" className='Question_logo' /></button>
      <div className='Car_Information' {...handlers}>
        {/* 차량 이미지 슬라이드 */}
        <img 
          src={carData.length > 0 ? carData[currentIndex]?.carType.image : CarSample} 
          alt={carData.length > 0 ? carData[currentIndex]?.carType.name : "Car Sample"} 
          className="Car_Image" 
        />
        
        <div className='Car_MonthInformation'>
  <div>
    <div className='Car_MonthInformation1'>배터리</div>
    <div className='MonthPayment_Contents'>
      <div>
        {/* 배터리 상태 */}
        {[...Array(10)].map((_, index) => {
          const batteryLevel = carData[currentIndex]?.battery || 0;
          let batteryClass = 'NoChargeBattery_img'; // 기본적으로 배터리가 없는 경우 클래스

          if (batteryLevel === 100) {
            batteryClass = 'ChargeBattery_img1'; // 100%일 경우
          } else if (batteryLevel >= 70) {
            batteryClass = 'ChargeBattery_img2'; // 70~99%일 경우
          } else if (batteryLevel >= 30) {
            batteryClass = 'ChargeBattery_img3'; // 30~69%일 경우
          } else if (batteryLevel > 0) {
            batteryClass = 'ChargeBattery_img4'; // 1~29%일 경우
          }

          return (
            <div 
              key={index} 
              className={batteryLevel >= (10 - index) * 10 
                ? batteryClass 
                : 'NoChargeBattery_img'} // 각 배터리 단계에 따라 클래스 적용
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
    </div>
    </>
  );

};

export default MainPage;
