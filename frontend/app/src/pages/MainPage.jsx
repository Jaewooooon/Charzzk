import React, { useState } from 'react';
import { Link } from 'react-router-dom';
import '../styles/MainPage.css';
import CarSample from '../assets/car_sample.png';
import MypageCar from '../assets/MypageCar.png';
import MyPagePayment from '../assets/MyPagePayment.png';
import MyPageReport from '../assets/MypageReport.png';
import ChargeStatus from '../assets/ChargeStatus.png';
import ChargeStart from '../assets/ChargeStart.png';

const MainPage = () => {
  const [menuHeight, setMenuHeight] = useState(500); // 기본 메뉴 높이
  const [isExpanded, setIsExpanded] = useState(false); // 메뉴가 전체화면인지
  const [startY, setStartY] = useState(0); // 터치 시작 위치
  const [startHeight, setStartHeight] = useState(500); // 터치 시작 시 높이

  const handleTouchStart = (e) => {
    setStartY(e.touches[0].clientY);
    setStartHeight(menuHeight);
  };

  const handleTouchMove = (e) => {
    const touchY = e.touches[0].clientY;
    const distance = startY - touchY; // 손가락 이동 거리
    const newHeight = startHeight + distance;

    if (newHeight >= 100 && newHeight <= window.innerHeight) {
      setMenuHeight(newHeight); // 손가락 위치에 따라 메뉴 높이 설정
    }
  };

  const handleTouchEnd = (e) => {
    const touchY = e.changedTouches[0].clientY;
    const distance = startY - touchY;

    // 스와이프 속도 계산 (distance / time)
    const duration = e.timeStamp - e.target.dataset.startTime;

    if (duration < 300 && distance > 50) {
      // 빠르게 스와이프했을 때 전체 화면 확장
      setIsExpanded(true);
      setMenuHeight(window.innerHeight);
    } else if (menuHeight > window.innerHeight *0.8) {
      // 천천히 스와이프 했지만 높이가 절반 이상일 때 전체 확장
      setIsExpanded(true);
      setMenuHeight(window.innerHeight);
    } else {
      // 그 외에는 원래 높이로 돌아감
      setIsExpanded(false);
      setMenuHeight(500);
    }
  };

  return (
    <div className='MainPage_ContainerBox'>
      <div className='Car_Information'>
        <img src={CarSample} alt="Car Sample" className="Car_Image" />
        <div className='Car_MonthInformation'>
          <div className='Car_MonthInformation1'>이번달 충전 요금</div>
          <div className='Car_MonthInformation2'>이번달 충전량</div>
        </div>
      </div>

      <div
        className={`MainPage_ButtonBox ${isExpanded ? 'expanded' : ''}`}
        style={{ height: `${menuHeight}px` }} // 높이를 동적으로 설정
        onTouchStart={handleTouchStart}
        onTouchMove={handleTouchMove}
        onTouchEnd={handleTouchEnd}
        data-start-time={Date.now()} // 스와이프 시작 시간
      >
        <div className='ChargePage_Content'>충전페이지</div>
        <div className='ChargePage'>
          <div className="ButtonContainer">
            <Link to="/charge-map">
              <button className='ChargeFind_Button'>
                <img src={ChargeStatus} alt="ChargeStatus" className="ChargeStatus_img" />
                충전구역 조회
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
              <img src={MypageCar} alt="MypageCar" className="MypageCar_Image" />
              차량 관리
            </button>
          </Link>

          <Link to="/mypage/payment-management">
            <button className='Mypage_Payment'>
              <img src={MyPagePayment} alt="MyPagePayment" className="MyPagePayment_Image" />
              결제수단 관리
            </button>
          </Link>

          <Link to="/mypage/report-issue">
            <button className='Mypage_ReportIssue'>
              <img src={MyPageReport} alt="MyPageReport" className="MyPageReport_Image" />
              신고하기
            </button>
          </Link>
        </div>
      </div>
    </div>
  );
};

export default MainPage;
