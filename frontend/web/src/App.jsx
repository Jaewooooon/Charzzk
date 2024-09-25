import React from "react";
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import './App.css';
import DocumentManagement from './pages/DocumentManagement';
import MapManagement from './pages/MapManagement';
import Navbar from "./components/Navbar";
import MainManagement from "./pages/MainManagement";
const App = () => {

  return (

      <Router>
        <Navbar />
        <Routes>
        <Route path = "/management" element={<MainManagement />} />
          <Route path = "/management/map-management" element={<MapManagement />} />
          <Route path = "/management/document-management" element={<DocumentManagement />} />
        </Routes>
      </Router>
  )
}

export default App;
