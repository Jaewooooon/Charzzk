import React from 'react';
import '../components/styles/ReadyNextButton.css';

function ReadyNextButton({ onClick }) {
  return (
    <div>
      <button className='Ready_NextButton' onClick={onClick}>
        다음
      </button>
      <div className='Ready_NextButtonBox'></div>
    </div>
  );
}

export default ReadyNextButton;
