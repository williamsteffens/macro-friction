#include "FBOWidget.h"

#include <QOpenGLFramebufferObject>
#include <QPainter>

FBOWidget::FBOWidget() :
    m_image()
{
    setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
}

void FBOWidget::setImage(const QImage& _image, int chan)
{
    m_image = _image;

    if( 0 <= chan && chan <= 3 )
    {
        for(int i = 0; i < m_image.width(); ++i)
        {
            for(int j = 0; j < m_image.height(); ++j)
            {
                QRgb c = m_image.pixel(i, j);
                switch(chan)
                {
                case 0:
                    c = qRgba(4*qRed(c),4*qRed(c),4*qRed(c),255);
                    break;
                case 1:
                    c = qRgba(4*qGreen(c),4*qGreen(c),4*qGreen(c),255);
                    break;
                case 2:
                    c = qRgba(4*qBlue(c),4*qBlue(c),4*qBlue(c),255);
                    break;
                case 3:
                    c = qRgba(4*qAlpha(c),4*qAlpha(c),4*qAlpha(c),255);
                    break;
                }
                m_image.setPixel(i,j, c);
            }
        }
    }
}

void FBOWidget::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    painter.drawImage(QRectF(0,0,this->width(), this->height()), m_image, QRectF(0,0, m_image.width(), m_image.height()));
    painter.drawRect(1,1,width()-2, height()-2);
    painter.end();
}

