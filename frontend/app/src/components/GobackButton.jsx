import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import '../components/styles/GobackButton.css'

function GoBackButton() {
  const navigate = useNavigate();

  const handleReturnClick = () => {
    navigate('/'); 
  };

  return (
  <div>
    <button className='ReportIssue_return_button' onClick={handleReturnClick}>x</button>

    </div>
  );
}

export default GoBackButton;
