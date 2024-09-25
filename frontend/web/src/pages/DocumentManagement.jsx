import { useState } from 'react'
import "../styles/DocumentManagement.css";

const DocumentManagement = () => {

  return (
    <div className='DocumentManagement_container'>
      <div className='IssueList_title'>신고내역</div>
      <div className='IssueList_Box'>
        <div className='IssueList_Contents_title'>
          <div> 신고시각</div>
          <div>작성자</div>
          <div>제목</div>
          <div>확인 여부</div>
        </div>
      </div>
    </div>
  )
}

export default DocumentManagement;
