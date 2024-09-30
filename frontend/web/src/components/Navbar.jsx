import React, { useState } from 'react';
import { NavLink, Outlet } from 'react-router-dom'; // Outlet을 사용하여 페이지 콘텐츠를 표시
import document from "../assets/Document_image.png";
import map from "../assets/Map_image.png";
import logo from "../assets/Web_Logo.png";
import "../styles/Navbar.css";

const Navbar = () => {
  const [activeButton, setActiveButton] = useState(null);

  const handleButtonClick = (buttonName) => {
    setActiveButton(buttonName);
  };

  const handleTopButtonClick = () => {
    setActiveButton(null);
  };

  return (
    <div className="main_layout">
      {/* 상단 네비게이션 바 */}
      <div className="top_nav">
      <NavLink to="/management">
        <button className='top_button' onClick={handleTopButtonClick}>
          <img src={logo} alt="logo" className="logo_img" /></button>
        </NavLink>
      </div>

      <div className='test'>
        {/* 좌측 네비게이션 바 */}
        <nav className="nav_container">
          <NavLink to="/management/map-management">
            <button
              className={`MapManagement_button ${activeButton === 'map' ? 'active' : ''}`}
              onClick={() => handleButtonClick('map')}
            >
              <img src={map} alt="map" className="map_img" />
            </button>
          </NavLink>
          <NavLink to="/management/document-management">
            <button
              className={`DocumentManagement_button ${activeButton === 'document' ? 'active' : ''}`}
              onClick={() => handleButtonClick('document')}
            >
              <img src={document} alt="document" className="document_img" />
            </button>
          </NavLink>
        </nav>

        {/* 메인 콘텐츠 영역 */}
        <div className="content">
          <Outlet /> {/* 페이지 콘텐츠가 표시되는 부분 */}
        </div>
      </div>
    </div>
  );
};

export default Navbar;
