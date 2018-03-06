/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "innerviewer.h"

InnerViewer::InnerViewer( InnerModelMgr innerModel_, const std::string &name_, uint period_, QObject *parent ) : period(period_)
{	
	stop = stopped = false;
	
	QGLFormat fmt;
	fmt.setDoubleBuffer(true);
	QGLFormat::setDefaultFormat(fmt);
	tb = new osgGA::TrackballManipulator;
	osg::Vec3d eye(osg::Vec3(4000., 4000., 1000.));
	osg::Vec3d center(osg::Vec3(0., 0., -0.));
	osg::Vec3d up(osg::Vec3(0., -1., 0.));
	tb->setHomePosition(eye, center, up, true);
	tb->setByMatrix(osg::Matrixf::lookAt(eye, center, up));
	viewer.setCameraManipulator(tb);
	osgViewer::Viewer::ThreadingModel threadingModel = osgViewer::Viewer::AutomaticSelection;
	viewer.setThreadingModel(threadingModel);
	viewer.addEventHandler(new osgViewer::WindowSizeHandler);
	createWindow(viewer, name_);
	osg::Group *root = new osg::Group();
	
	viewer.getLight()->setPosition(osg::Vec4(1,-1, 1, 0)); // make 4th coord 1 for point
	viewer.getLight()->setAmbient(osg::Vec4(0.2, 0.2, 0.2, 1.0));
	//viewer.getLight()->setDiffuse(osg::Vec4(0.7, 0.4, 0.6, 1.0));
	viewer.getLight()->setSpecular(osg::Vec4(1.0, 1.0, 1.0, 1.0));

	qDebug() << "----------------------------------In viewer" << innerModel_->getNode<InnerModelJoint>("armX1")->getAngle();
	
	InnerModelMgr in = innerModel_.deepcopy();
	innerModel = innerModel_.deepcopy();
	
	qDebug() << "---------------------------------In innerModel_" << innerModel_->getNode<InnerModelJoint>("armX1")->getAngle();
	
	qDebug() << "---------------------------------In innerModel" << innerModel->getNode<InnerModelJoint>("armX1")->getAngle();
 		
	qDebug() << "----------------------------------In  in" << in->getNode<InnerModelJoint>("armX1")->getAngle();
	
	if (innerModel_->getNode<InnerModelJoint>("armX1")->getAngle() != innerModel->getNode<InnerModelJoint>("armX1")->getAngle())
		//qFatal("Me cago en el Fary");
	
	
	
	innerModelViewer = new InnerModelViewer(innerModel, "root", root, true);
	viewer.setSceneData(root);
	
	//////////////////////////
	//RESTORE FORMER VIEW					QUEDA CAPTURAR EL EVENTO DE CIERRE DE LA VENTANA PARA GUARDAR LA MATRIZ ACTUAL
	/////////////////////////
 	settings = new QSettings("RoboComp", "InnerViewer");
	QString path(".");
	QStringList l = settings->value("matrix").toStringList();
	if (l.size() > 0)
	{
		osg::Matrixd m;
		for (int i=0; i<4; i++ )
			for (int j=0; j<4; j++ )
				m(i,j)=l.takeFirst().toDouble();
		tb->setByMatrix(m);
	}
		else
			innerModelViewer->setMainCamera(tb, InnerModelViewer::TOP_POV);
	 	
	viewer.realize();
}

void InnerViewer::run()
{
	while(true)
	{
		{
			guard gl(mutex);
			innerModelViewer->update();   //accesses local InnerModel for reading and the osg scenegraph
			viewer.frame();
		}
		
		usleep(period);	

		while (stop)
		{
			stopped = true;
			usleep(period);	
		}
		stopped = false;
		
	}
}

void InnerViewer::reloadInnerModel(InnerModelMgr other)
{	

	qDebug()<<"reloadInnerModel-----------0-----------";
	guard gl(mutex);
	qDebug()<<"reloadInnerModel-----------1-----------";
 	innerModelViewer-> innerModel = other.deepcopy(); 	
	qDebug()<<"reloadInnerModel-----------2-----------";
 	//innerModelViewer->innerModel->print("");
	qDebug()<<"reloadInnerModel-----------2.5-----------"<< stopped;
//  	innerModelViewer->update();  
// 	qDebug()<<"reloadInnerModel-----------3-----------";
// 	viewer.frame();
// 	qDebug()<<"reloadInnerModel-----------4-----------";
}

void InnerViewer::updateTransformValues(const QString item, const QVec &pos, const QString &parent)
{
	guard gl(mutex);
	innerModel->updateTransformValues(item, pos.x(), pos.y(), pos.z(), pos.rx(), pos.ry(), pos.rz(), parent);
}

//////////////////////////////////////////////////////
//// Non thread saffe API 
//////////////////////////////////////////////////////

void InnerViewer::removeNode(const QString &item_)
{	
	//preconditions
	InnerModelNode *node = innerModel->getNode(item_);
	if (node == NULL)
		throw QString("InnerViewer::remove node: Can't remove not existing element " + item_);

	if (item_ == "root")
		throw QString("InnerViewer::remove node: Can't remove root lement " + item_);

	QStringList l;
	innerModelViewer->innerModel->getSubTree(node, &l);
	innerModelViewer->innerModel->removeSubTree(node, &l);

	/// Replicate InnerModel node removals in the InnerModelViewer tree. And in handlers Lists
	foreach(const QString &n, l)
	{
		/// Replicate mesh removals
		if (innerModelViewer->meshHash.contains(n))
		{
			while (innerModelViewer->meshHash[n].osgmeshPaths->getNumParents() > 0)
				innerModelViewer->meshHash[n].osgmeshPaths->getParent(0)->removeChild(innerModelViewer->meshHash[n].osgmeshPaths);
			while(innerModelViewer->meshHash[n].osgmeshes->getNumParents() > 0)
				innerModelViewer->meshHash[n].osgmeshes->getParent(0)->removeChild(innerModelViewer->meshHash[n].osgmeshes);
			while(innerModelViewer->meshHash[n].meshMts->getNumParents() > 0)
				innerModelViewer->meshHash[n].meshMts->getParent(0)->removeChild(innerModelViewer->meshHash[n].meshMts);
			innerModelViewer->meshHash.remove(n);
		}
		/// Replicate transform removals
		if (innerModelViewer->mts.contains(n))
		{
 			while (innerModelViewer->mts[n]->getNumParents() > 0)
				innerModelViewer->mts[n]->getParent(0)->removeChild(innerModelViewer->mts[n]);
 			innerModelViewer->mts.remove(n);
		}
		/// Replicate plane removals
		if (innerModelViewer->planeMts.contains(n))
		{
			while(innerModelViewer->planeMts[n]->getNumParents() > 0)
				((osg::Group *)(innerModelViewer->planeMts[n]->getParent(0)))->removeChild(innerModelViewer->planeMts[n]);
			innerModelViewer->planeMts.remove(n);
			innerModelViewer->planesHash.remove(n);
		}
	}
}

void InnerViewer::addTransform_ignoreExisting(const QString &item_, const QString &parent_, const QVec &pos)
{
	//preconditions
	if(pos.size() != 6)
		throw QString("InnerViewer::addPlane_ignoreExisting: Position vector has not dimension 6");
	
	InnerModelNode *parent = innerModelViewer->innerModel->getNode(parent_);
	if (parent == NULL)
		throw QString("InnerViewer::addTransform_ignoreExisting: parent element node doesn't exist");

	if (innerModelViewer->innerModel->getNode(item_) != NULL)
		this->removeNode(item_);

	this->addTransform(item_, parent_, pos);
}

void InnerViewer::addTransform(const QString &item_, const QString &parent_,const QVec &pos)
{
	//preconditions
	if(pos.size() != 6)
		throw QString("InnerViewer::addPlane_ignoreExisting: Position vector has not dimension 6");
	
	InnerModelNode *parent = innerModelViewer->innerModel->getNode(parent_);
	if (parent == NULL)
		throw QString("InnerViewer::addTransform: parent node doesn't exist");

	InnerModelNode *node = innerModelViewer->innerModel->getNode(item_);
	if (node != NULL)
		throw QString("InnerViewer::addTransform: item alreaddy exists");

	InnerModelTransform *tr;
	try
	{
		tr = innerModelViewer->innerModel->newTransform(item_, "static", parent, 0,0,0, 0,0,0);
		parent->addChild(tr);
		innerModelViewer->recursiveConstructor(tr, innerModelViewer->mts[parent->id], innerModelViewer->mts, innerModelViewer->meshHash);
		innerModelViewer->innerModel->updateTransformValues(item_, pos.x(), pos.y(), pos.z(), pos.rx(), pos.ry(), pos.rz());
	}
	catch (QString err)
	{
		printf("%s:%s:%d: Exception: %s\n", __FILE__, __FUNCTION__, __LINE__, err.toStdString().c_str());
		throw err;
	}
}

void InnerViewer::drawLine(const QString &item_, const QString &parent_, const QVec &center, const QVec &normal, float length, float width, const QString &texture)
{
	this->addPlane_ignoreExisting(item_, parent_, center, normal, texture, QVec::vec3(length, width, width));
}

void InnerViewer::addPlane_ignoreExisting(const QString &item_, const QString &parent_, const QVec &center, const QVec &normal, const QString &texture, const QVec &size)
{
	InnerModelNode *parent = innerModel->getNode(parent_);
	if (parent == NULL)
		throw QString("InnerViewer::addPlane_ignoreExisting: parent element node doesn't exist " + item_);
	InnerModelPlane *plane = innerModelViewer->innerModel->newPlane(item_, parent, texture, size(0), size(1), size(2), 1, normal(0), normal(1), normal(2), center(0), center(1), center(2));
	parent->addChild(plane);
	innerModelViewer->recursiveConstructor(plane, innerModelViewer->mts[parent->id], innerModelViewer->mts, innerModelViewer->meshHash);
}

void InnerViewer::addPlane_notExisting(const QString &item_, const QString &parent_, const QVec &center, const QVec &normal, const QString &texture, const QVec &size)
{
	InnerModelNode *parent = innerModel->getNode(parent_);
	if (parent == NULL)
		throw QString("InnerViewer::addPlane_notExisting: parent node doesn't exist");
	InnerModelPlane *plane = innerModel->newPlane(item_, parent, texture, size(0), size(1), size(2), 1, normal(0), normal(1), normal(2), center(0), center(1), center(2));
	parent->addChild(plane);
	innerModelViewer->recursiveConstructor(plane, innerModelViewer->mts[parent->id], innerModelViewer->mts, innerModelViewer->meshHash);
}
///////////////////////////////////////////////////////////////////////////////////77

// UNTIL we know how to capture the close window signal from OSG/X11
// 			osg::Matrixd m = tb->getMatrix();
// 			QString s="";
// 			QStringList l;
// 			for (int i=0; i<4; i++ )
// 				for (int j=0; j<4; j++ )
// 	 			l.append(s.number(m(i,j)));
// 			settings->setValue("matrix", l);
// 			settings->sync();

/////////////////////////////////////
/// Auxiliary methods
/////////////////////////////////////
void InnerViewer::createWindow(osgViewer::Viewer& viewer, const std::string &name)
{
	osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();
	if (!wsi)
	{
		osg::notify(osg::NOTICE)<<"Error, no WindowSystemInterface available, cannot create windows."<<std::endl;
		return;
	}

	unsigned int width, height;
	wsi->getScreenResolution(osg::GraphicsContext::ScreenIdentifier(0), width, height);

	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	traits->x = 0;
	traits->y = 0;
	traits->width = 700;
	traits->height = 700;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->windowName = "InnerModelViewer " + name;

	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	if (gc.valid())
	{
		osg::notify(osg::INFO)<<"  GraphicsWindow has been created successfully."<<std::endl;

		// need to ensure that the window is cleared make sure that the complete window is set the correct colour
		// rather than just the parts of the window that are under the camera's viewports
		gc->setClearColor(osg::Vec4f(0.2f,0.2f,0.6f,1.0f));
		gc->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}
	else
	{
		osg::notify(osg::NOTICE)<<"  GraphicsWindow has not been created successfully."<<std::endl;
	}
	unsigned int numCameras = 1;
	double aspectRatioScale = 1.0;///(double)numCameras;
	for(unsigned int i=0; i<numCameras;++i)
	{
		osg::ref_ptr<osg::Camera> camera = new osg::Camera;
		camera->setGraphicsContext(gc.get());
		camera->setViewport(new osg::Viewport((i*width)/numCameras,(i*height)/numCameras, width/numCameras, height/numCameras));
		GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
		camera->setDrawBuffer(buffer);
		camera->setReadBuffer(buffer);

		viewer.addSlave(camera.get(), osg::Matrixd(), osg::Matrixd::scale(aspectRatioScale,1.0,1.0));
	}
}

