import React from "react";
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import './App.css';
import DocumentManagement from './pages/DocumentManagement';
import MapManagement from './pages/MapManagement';
import Navbar from "./components/Navbar";
const App = () => {

  return (

      <Router>
        <Navbar />
        <Routes>
          <Route path = "/" element={<MapManagement />} />
          <Route path = "/document-management" element={<DocumentManagement />} />
        </Routes>
      </Router>
  )
}

export default App;
