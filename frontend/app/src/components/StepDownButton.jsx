import React from 'react';
import '../components/styles/StepDownButton.css';
import BackButton from '../assets/BackButton.png';

const StepDownButton = ({ onClick }) => {
  return (
    <button onClick={onClick} className='step-down-button'>
      <img src={BackButton} alt="이전" className='Backimg'/>
    </button>
  );
};

export default StepDownButton;