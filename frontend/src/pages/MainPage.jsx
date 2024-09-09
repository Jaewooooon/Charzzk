import React from 'react';
import { Link } from 'react-router-dom';  // Link 컴포넌트 임포트
import '../styles/MainPage.css';

const MainPage = () => {
  return (
    <div className='MainPage_ContainerBox'>
      <>차정보</>
    <div className='MainPage_ButtonBox'>
        <div>
          <Link to="/charge-map">
            <button>충전구역 조회</button>
          </Link>
        </div>
        <div>
          <Link to="/charge-start">
            <button>충전하기</button>
          </Link>
        </div>
        <div>
          <Link to="/charge-status">
            <button>충전현황 확인</button>
          </Link>
        </div>
        <div>
          <Link to="/mypage/car-management">
            <button>차량 관리</button>
          </Link>
        </div>
        <div>
          <Link to="/mypage/payment-management">
            <button>결제수단 관리</button>
          </Link>
        </div>
        <div>
          <Link to="/mypage/report-issue">
            <button>신고하기</button>
          </Link>
        </div>
    </div>
    </div>
  );
}

export default MainPage;
