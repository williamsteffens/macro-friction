#include "EllipseWidget.h"

#include <QOpenGLFramebufferObject>
#include <QPainter>

FrictionLimitsWidget::FrictionLimitsWidget() : QWidget()
{
    setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);

    memset(m_coeffs, 0, sizeof(CoeffsType));
}

void FrictionLimitsWidget::setCoeffs(const CoeffsType& _coeffs)
{
    memcpy(m_coeffs, _coeffs, sizeof(CoeffsType));
}

void FrictionLimitsWidget::paintEvent(QPaintEvent* event)
{
    QPen pen;
    pen.setWidth(2);
    pen.setBrush(Qt::SolidPattern);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen( QPen(Qt::blue, 2.0) );

    const int cx = width() / 2;
    const int cy = height() / 2;
    const int xscale = 0.25 * width();
    const int yscale = 0.25 * height();
    const int left = cx - m_coeffs[2]*xscale;
    const int top = cy - m_coeffs[1]*yscale;
    const int width = (m_coeffs[0] + m_coeffs[2])*xscale;
    const int height = (m_coeffs[1] + m_coeffs[3])*yscale;
    QRect rect(left, top, width, height);
    painter.drawRect(rect);


    painter.setPen( QPen(Qt::red, 1.0) );
    painter.drawLine(cx-10, cy, cx+10, cy);
    painter.setPen( QPen(Qt::green, 1.0) );
    painter.drawLine(cx, cy-10, cx, cy+10);
    painter.end();
}

