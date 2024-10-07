import React from "react";
import '../components/styles/CompleteChargeModal.css';

const CompleteChargeModal = ({ isOpen, onClose, children }) => {
  if (!isOpen) return null;

  return (
    <div className="modal-overlay" onClick={onClose}>
      <div className="modal-content" onClick={e => e.stopPropagation()}>
        {children}
      </div>
    </div>
  );
};
export default CompleteChargeModal;
